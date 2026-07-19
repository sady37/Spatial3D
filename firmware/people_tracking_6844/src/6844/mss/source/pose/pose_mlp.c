/*
 * pose_mlp.c -- forward pass + per-track feature buffering for the 6844 pose MLP.
 *
 * Weights: pose_model.c (gPoseW0..3 / gPoseB0..3), BatchNorm pre-folded.
 * Reference implementation and the exported arrays are validated bit-for-bit
 * against PyTorch in pc/pose/train.py (max |torch - folded| < 3e-7).
 *
 * Feature vector (per frame, 20 wide), matching pc/pose/dataset.py FEATURE_NAMES:
 *   [0] posz            track height, remapped to TI's reference (raw + zOffset)
 *   [1] vely            track velocity Y
 *   [2] velz            track velocity Z
 *   [3] accy            track accel Y
 *   [4] accz            track accel Z
 *   [5..19] 5x (pointY - posY, pointZ, snr) for the 5 highest gated points,
 *           ordered lowest-of-the-top-5 first (ascending height) -- identical
 *           to TI's df_points.sort_values('pointz').tail(5).
 *
 * The 8-frame window is stored feature-major to match CreateFeatureVector():
 *   input[j*8 + k] = frame k (k=0 oldest), feature j.
 */

#include <string.h>
#include <math.h>

#include "pose_mlp.h"

/* Folded weights (generated). Row-major: W[out][in]. */
extern const float gPoseW0[64][POSE_INPUT_SIZE];
extern const float gPoseB0[64];
extern const float gPoseW1[32][64];
extern const float gPoseB1[32];
extern const float gPoseW2[16][32];
extern const float gPoseB2[16];
extern const float gPoseW3[POSE_NUM_CLASSES][16];
extern const float gPoseB3[POSE_NUM_CLASSES];

/* Only associate points within this radius of a target (metres). Generalises
 * TI's single-track "all points" to multi-track; loose enough to keep a body's
 * spread, tight enough to not steal a neighbour's points. */
#define POSE_GATE_RADIUS_M   0.75f
#define POSE_GATE_RADIUS_SQ  (POSE_GATE_RADIUS_M * POSE_GATE_RADIUS_M)

/* Drop a track's buffer after this many frames without an update (stale tid). */
#define POSE_STALE_FRAMES    15

/* Frame rate for the point-centroid kinematics (velZ = d(centroid)/frame * POSE_FPS). 10 fps,
 * matching pc/pose/train_6844.py FPS so train == on-chip. */
#define POSE_FPS             10.0f

/* Pin pose .bss (~6.3 KB) to TCMB. It must NOT land in TCMA: after the cubeQuery
 * patch TCMA has only ~5.8 KB free, and the general .bss spills
 * ">> TCMA_RAM | TCMB_RAM", so an unpinned 6.3 KB could tip TCMA and brick boot
 * (same failure as fallsm-boot-bug). linker.cmd routes .bss.pose -> TCMB_RAM. */
#ifndef POSE_TCMB   /* host tests override with -DPOSE_TCMB= (mach-o rejects the name) */
#define POSE_TCMB __attribute__((section(".bss.pose")))
#endif

/* ---- per-track ring buffer state (.bss) ------------------------------------
 * feat[k] is frame k's 20-feature vector, k=0 oldest. A slot is "full" once
 * POSE_NUM_FRAMES frames have been pushed since it was (re)claimed. */
typedef struct PoseSlot_t
{
    uint32_t tid;
    uint8_t  used;
    uint8_t  count;      /* frames buffered since claim, saturates at NUM_FRAMES */
    uint8_t  age;        /* frames since last update */
    uint8_t  pad;
    /* window (sustained-down) leg state */
    uint8_t  winLowRun;  /* consecutive low frames (saturates at sustain) */
    uint8_t  winHiRun;   /* consecutive high frames (for clear) */
    uint8_t  winDown;    /* latched down-state */
    uint8_t  pad2;
    /* previous frame's point-CENTROID kinematics, for the ring-diff velZ/accZ (the fall signal
     * the tracker smooths away). Reset to 0 when the slot is (re)claimed. */
    float    prevCz, prevCy, prevVz, prevVy;
    float    feat[POSE_NUM_FRAMES][POSE_NUM_FEATURES];
} PoseSlot;

POSE_TCMB static PoseSlot gPoseSlots[POSE_MAX_TRACKS];
static float    gPoseZOffset = 0.0f;

/* window leg floor geometry + timing (PoseMlp_setWindowCfg). Defaults mirror
 * pc/falldet/window.py. cos/sin precomputed from tilt. */
static float   gWinMountM  = 1.0f;
static float   gWinCosTilt = 1.0f;
static float   gWinSinTilt = 0.0f;
static float   gWinMarginM = 0.45f;
static uint8_t gWinSustain = 5;
static uint8_t gWinClear   = 5;

/* Scratch, reused per inference. Kept static to stay off the (small) stack. */
POSE_TCMB static float gPoseIn[POSE_INPUT_SIZE];
POSE_TCMB static float gPoseH0[64];
POSE_TCMB static float gPoseH1[32];
POSE_TCMB static float gPoseH2[16];
POSE_TCMB static float gPoseOut[POSE_NUM_CLASSES];

void PoseMlp_init(void)
{
    memset(gPoseSlots, 0, sizeof(gPoseSlots));
}

void PoseMlp_setZOffset(float zOffset)
{
    gPoseZOffset = zOffset;
}

void PoseMlp_setWindowCfg(float mountM, float tiltRad, float marginM,
                          uint8_t sustain, uint8_t clear)
{
    gWinMountM  = mountM;
    gWinCosTilt = cosf(tiltRad);
    gWinSinTilt = sinf(tiltRad);
    gWinMarginM = marginM;
    gWinSustain = (sustain > 0) ? sustain : 1;
    gWinClear   = (clear   > 0) ? clear   : 1;
}

/* dst = relu?(W*src + b). W is [outN][inN] row-major. */
static void poseDense(const float *W, const float *b, const float *src,
                      float *dst, int outN, int inN, int relu)
{
    int o, i;
    for (o = 0; o < outN; o++)
    {
        const float *w = W + (uint32_t)o * inN;
        float acc = b[o];
        for (i = 0; i < inN; i++)
        {
            acc += w[i] * src[i];
        }
        if (relu && acc < 0.0f)
        {
            acc = 0.0f;
        }
        dst[o] = acc;
    }
}

/* Run the folded net on gPoseIn -> gPoseOut (softmax). */
static void poseInfer(void)
{
    int c;
    float mx, sum;

    /* pose_model_6844.c is a scikit-learn MLP (ReLU after EVERY hidden layer) with the input
     * StandardScaler folded into layer 0 -> firmware feeds RAW point-centroid features. Layer 2
     * (16-unit) therefore has ReLU too (the old torch export skipped it). See pc/pose/train_6844.py. */
    poseDense(&gPoseW0[0][0], gPoseB0, gPoseIn, gPoseH0, 64, POSE_INPUT_SIZE, 1);
    poseDense(&gPoseW1[0][0], gPoseB1, gPoseH0, gPoseH1, 32, 64, 1);
    poseDense(&gPoseW2[0][0], gPoseB2, gPoseH1, gPoseH2, 16, 32, 1); /* relu (sklearn: all hidden) */
    poseDense(&gPoseW3[0][0], gPoseB3, gPoseH2, gPoseOut, POSE_NUM_CLASSES, 16, 0);

    mx = gPoseOut[0];
    for (c = 1; c < POSE_NUM_CLASSES; c++)
    {
        if (gPoseOut[c] > mx) mx = gPoseOut[c];
    }
    sum = 0.0f;
    for (c = 0; c < POSE_NUM_CLASSES; c++)
    {
        gPoseOut[c] = expf(gPoseOut[c] - mx);
        sum += gPoseOut[c];
    }
    if (sum > 0.0f)
    {
        for (c = 0; c < POSE_NUM_CLASSES; c++) gPoseOut[c] /= sum;
    }
}

/* Find/claim the ring-buffer slot for a tid. NULL if the table is full. */
static PoseSlot *poseSlotFor(uint32_t tid)
{
    int i, free = -1;
    for (i = 0; i < POSE_MAX_TRACKS; i++)
    {
        if (gPoseSlots[i].used && gPoseSlots[i].tid == tid)
        {
            return &gPoseSlots[i];
        }
        if (free < 0 && !gPoseSlots[i].used)
        {
            free = i;
        }
    }
    if (free < 0)
    {
        return NULL;
    }
    memset(&gPoseSlots[free], 0, sizeof(PoseSlot));
    gPoseSlots[free].used = 1;
    gPoseSlots[free].tid  = tid;
    return &gPoseSlots[free];
}

/* Build one frame's 20-feature vector for a target into dst[POSE_NUM_FEATURES].
 * Reads points in place through getPt (no copy). Returns 1 on success, 0 if
 * fewer than POSE_NUM_POINTS points gate to it. */
static int poseBuildFrame(const PoseTrackKin *k, const void *ptsCtx,
                          uint32_t numPoints, PosePointGet getPt, float *dst)
{
    /* Top-5 highest gated points, kept as a size-5 ascending-by-z heap.
     * top[0] is the lowest of the current top-5 (the one to evict). */
    float topZ[POSE_NUM_POINTS];
    float topY[POSE_NUM_POINTS];   /* pointY - posY */
    float topS[POSE_NUM_POINTS];   /* snr */
    int   n = 0;
    uint32_t p;
    int i;

    for (p = 0; p < numPoints; p++)
    {
        float px, py, pz, s;
        getPt(ptsCtx, p, &px, &py, &pz, &s);
        float dx = px - k->posX;
        float dy = py - k->posY;
        if (dx * dx + dy * dy > POSE_GATE_RADIUS_SQ)
        {
            continue;
        }
        float z  = pz;
        float yr = py - k->posY;

        if (n < POSE_NUM_POINTS)
        {
            /* insert, keep ascending by z */
            i = n++;
            while (i > 0 && topZ[i - 1] > z)
            {
                topZ[i] = topZ[i - 1]; topY[i] = topY[i - 1]; topS[i] = topS[i - 1];
                i--;
            }
            topZ[i] = z; topY[i] = yr; topS[i] = s;
        }
        else if (z > topZ[0])
        {
            /* evict lowest (index 0), insert keeping ascending order */
            i = 0;
            while (i < POSE_NUM_POINTS - 1 && topZ[i + 1] < z)
            {
                topZ[i] = topZ[i + 1]; topY[i] = topY[i + 1]; topS[i] = topS[i + 1];
                i++;
            }
            topZ[i] = z; topY[i] = yr; topS[i] = s;
        }
    }

    if (n < POSE_NUM_POINTS)
    {
        return 0;
    }

    /* ⭐ POINT-CENTROID kinematics -- NOT the tracker's. Track-Z floats on a still body and the
     * EKF smooths velZ through a fall, so k->posZ/velZ read ~constant/0 and the MLP only ever
     * emitted Stood (Lying/Falling dead). The gated POINTS carry the true vertical signal.
     * dst[0]=centroid Z (mean top-5), dst[1]=centroid Y (TEMP: the caller converts dst[1..4] into
     * velY/velZ/accY/accZ from the previous frame's centroid in the slot). snr is zeroed to match
     * training (the recorded cloud has no per-point snr). See pc/pose/train_6844.py -- train==this. */
    {
        float czm = 0.0f, cym = 0.0f;
        for (i = 0; i < POSE_NUM_POINTS; i++) { czm += topZ[i]; cym += topY[i]; }
        dst[0] = czm / (float)POSE_NUM_POINTS;   /* centroid Z (replaces track posZ) */
        dst[1] = cym / (float)POSE_NUM_POINTS;   /* TEMP centroid Y -> caller makes velY */
        dst[2] = 0.0f;                            /* velZ -- caller fills from the ring */
        dst[3] = 0.0f;                            /* accY -- caller fills */
        dst[4] = 0.0f;                            /* accZ -- caller fills */
    }
    for (i = 0; i < POSE_NUM_POINTS; i++)
    {
        dst[5 + i * 3 + 0] = topY[i];
        dst[5 + i * 3 + 1] = topZ[i];
        dst[5 + i * 3 + 2] = 0.0f;               /* snr=0 (training used snr=0; npz cloud has none) */
    }
    (void)topS;
    return 1;
}

/* Push a fresh frame into the slot's ring (shift down, newest at index count-1). */
static void posePush(PoseSlot *s, const float *frame)
{
    int k;
    for (k = 0; k < POSE_NUM_FRAMES - 1; k++)
    {
        memcpy(s->feat[k], s->feat[k + 1], sizeof(s->feat[0]));
    }
    memcpy(s->feat[POSE_NUM_FRAMES - 1], frame, sizeof(s->feat[0]));
    if (s->count < POSE_NUM_FRAMES)
    {
        s->count++;
    }
    s->age = 0;
}

/* Flatten the ring buffer feature-major into gPoseIn: in[j*8 + k] = feat[k][j]. */
static void poseFlatten(const PoseSlot *s)
{
    int k, j;
    for (j = 0; j < POSE_NUM_FEATURES; j++)
    {
        for (k = 0; k < POSE_NUM_FRAMES; k++)
        {
            gPoseIn[j * POSE_NUM_FRAMES + k] = s->feat[k][j];
        }
    }
}

/* Window (sustained-down) leg for one track. Reuses the same gate + accessor as
 * the MLP. Computes the 2nd-highest world-height above the floor over the gated
 * points (robust to a single ghost spike), then runs the K-frame sustain on the
 * slot. Track-independent of kinematics (uses point heights, not posZ), so it
 * survives the track freeze/ghost that breaks the MLP during a fall. Needs >=2
 * gated points. Writes h_s (cm) and validity via hsCmOut and validOut.
 * Ports pc/falldet/window.py WindowDetector.update(). */
static void poseWindowUpdate(PoseSlot *s, const PoseTrackKin *k,
                             const void *ptsCtx, uint32_t numPoints,
                             PosePointGet getPt, int16_t *hsCmOut, uint8_t *validOut)
{
    float h1 = -1e30f, h2 = -1e30f;   /* highest, 2nd-highest world height */
    int   n = 0;
    uint32_t p;

    for (p = 0; p < numPoints; p++)
    {
        float px, py, pz, snr;
        getPt(ptsCtx, p, &px, &py, &pz, &snr);
        float dx = px - k->posX;
        float dy = py - k->posY;
        if (dx * dx + dy * dy > POSE_GATE_RADIUS_SQ)
        {
            continue;
        }
        /* world height above floor: same transform the server uses */
        float h = gWinMountM + pz * gWinCosTilt - py * gWinSinTilt;
        if (h > h1)      { h2 = h1; h1 = h; }
        else if (h > h2) { h2 = h; }
        n++;
    }

    if (n < 2)
    {
        *validOut = 0;
        *hsCmOut  = 0;
        /* No measurement this frame: hold the latched state, don't advance runs. */
        return;
    }

    *validOut = 1;
    *hsCmOut  = (int16_t)(h2 * 100.0f);   /* metres -> cm */

    if (h2 <= gWinMarginM)
    {
        if (s->winLowRun < 255) s->winLowRun++;
        s->winHiRun = 0;
        if (s->winLowRun >= gWinSustain) s->winDown = 1;
    }
    else
    {
        if (s->winHiRun < 255) s->winHiRun++;
        s->winLowRun = 0;
        if (s->winHiRun >= gWinClear) s->winDown = 0;
    }
}

uint32_t PoseMlp_process(const PoseTrackKin *kin, uint32_t numTargets,
                         const void *ptsCtx, uint32_t numPoints,
                         PosePointGet getPoint, PoseResult *out)
{
    uint32_t t, written = 0;
    int i, c, best;
    float frame[POSE_NUM_FEATURES];

    /* Age every slot; updated ones reset to 0 below. */
    for (i = 0; i < POSE_MAX_TRACKS; i++)
    {
        if (gPoseSlots[i].used && ++gPoseSlots[i].age > POSE_STALE_FRAMES)
        {
            gPoseSlots[i].used = 0;   /* stale tid -> free the slot */
        }
    }

    for (t = 0; t < numTargets && written < POSE_MAX_TRACKS; t++)
    {
        PoseSlot *s = poseSlotFor(kin[t].tid);
        PoseResult *r = &out[written++];
        r->tid = kin[t].tid;
        r->pose = POSE_UNKNOWN;
        r->fallingProb = 0;
        r->valid = 0;
        r->winDown = 0;
        r->winHsCm = 0;
        r->winLowRun = 0;
        r->winValid = 0;

        if (s == NULL)
        {
            continue;   /* table full */
        }

        /* --- window leg: runs every frame, needs only >=2 gated points and no
         * ring/kinematics, so it survives the track freeze during a fall. --- */
        poseWindowUpdate(s, &kin[t], ptsCtx, numPoints, getPoint,
                         &r->winHsCm, &r->winValid);
        r->winDown   = s->winDown;
        r->winLowRun = s->winLowRun;

        /* --- MLP leg: needs >=5 gated points and a full 8-frame ring. --- */
        if (!poseBuildFrame(&kin[t], ptsCtx, numPoints, getPoint, frame))
        {
            /* Not enough points this frame: keep the buffer, no push. */
            continue;
        }
        /* Point-centroid kinematics from the slot's previous centroid (train_6844.py:
         * vel=(c-cprev)*fps, acc=(vel-vprev)*fps). A freshly-claimed slot (count==0) has no
         * previous frame -> zeros. frame[0]=cz, frame[1]=cy(temp) came from poseBuildFrame. */
        {
            float cz = frame[0], cy = frame[1];
            if (s->count >= 1)
            {
                float vz = (cz - s->prevCz) * POSE_FPS;
                float vy = (cy - s->prevCy) * POSE_FPS;
                frame[2] = vz;                           /* velZ */
                frame[1] = vy;                           /* velY (overwrite the temp centroid Y) */
                frame[4] = (vz - s->prevVz) * POSE_FPS;  /* accZ */
                frame[3] = (vy - s->prevVy) * POSE_FPS;  /* accY */
            }
            else
            {
                frame[1] = frame[2] = frame[3] = frame[4] = 0.0f;
            }
            s->prevCz = cz; s->prevCy = cy;
            s->prevVz = frame[2]; s->prevVy = frame[1];
        }
        posePush(s, frame);

        if (s->count < POSE_NUM_FRAMES)
        {
            continue;   /* need a full window before first inference */
        }

        poseFlatten(s);
        poseInfer();

        best = 0;
        for (c = 1; c < POSE_NUM_CLASSES; c++)
        {
            if (gPoseOut[c] > gPoseOut[best]) best = c;
        }
        r->pose = (uint8_t)best;
        r->fallingProb = (uint8_t)(gPoseOut[POSE_FALLING] * 255.0f + 0.5f);
        r->valid = 1;
    }

    return written;
}
