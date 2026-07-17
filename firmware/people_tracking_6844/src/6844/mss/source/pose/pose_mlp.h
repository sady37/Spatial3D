/*
 * pose_mlp.h -- Spatial3D per-track pose classifier for 6844 People_Tracking.
 *
 * A 4-class posture MLP (Stood / Sat / Lying / Falling) retrained from TI's
 * Pose_And_Fall classes.zip and ported to run natively on the R5F MSS -- no
 * TVM, no pose_model.a (that archive is Cortex-M4 / v7E-M Thumb-only and will
 * not link into this ARMv7-R image; see the Phase 2 notes in the version
 * README).  BatchNorm is folded into the Linear layers at export time, so
 * inference is pure multiply-add + relu + softmax.
 *
 * Topology : 160 -> 64 -> 32 -> 16 -> 4   (relu after layers 0,1 only)
 * Input    : 20 features x 8 frames = 160, feature-major
 *            in[j*8 + k] = frame k (k=0 oldest), feature j
 * Features : posz, vely, velz, accy, accz,
 *            then 5x (pointY-posY, pointZ, snr) for the 5 highest points.
 *
 * Weights live in pose_model.c (.rodata.pose_model -> TCMB_RAM, ~51 KB).
 * Runtime state (per-track ring buffers + scratch) is ~6.3 KB of .bss, pinned
 * to TCMB via the .bss.pose section; see POSE_MAX_TRACKS.
 */

#ifndef POSE_MLP_H_
#define POSE_MLP_H_

#include <stdint.h>

#define POSE_NUM_FEATURES     20
#define POSE_NUM_FRAMES        8
#define POSE_INPUT_SIZE       (POSE_NUM_FEATURES * POSE_NUM_FRAMES)   /* 160 */
#define POSE_NUM_CLASSES       4
#define POSE_NUM_POINTS        5      /* highest-N points used as features    */

/* Class ids -- match the server contract and the training enum. */
#define POSE_STOOD    0
#define POSE_SAT      1
#define POSE_LYING    2
#define POSE_FALLING  3
#define POSE_UNKNOWN  0xFF           /* buffer not yet full / no valid points */

/* Max simultaneously-classified tracks. The tracker caps well under this;
 * extra tracks beyond POSE_MAX_TRACKS simply get no pose this frame. */
#define POSE_MAX_TRACKS        8

/* One track's fall/pose result for the frame. Carries BOTH on-chip triggers
 * (the server OR-fuses them, then cleans with the cube second-check -- see
 * pc/falldet/clean.py):
 *   - MLP leg   : pose + fallingProb (falling MOTION + free pose)
 *   - window leg: winDown + winHsCm  (sustained DOWN-state, robust to track
 *                 freeze -- 2nd-highest point sits near the local floor for K
 *                 frames). Ports pc/falldet/window.py WindowDetector on-chip. */
typedef struct PoseResult_t
{
    uint32_t tid;          /* track id this result belongs to               */
    uint8_t  pose;         /* POSE_STOOD..POSE_FALLING, or POSE_UNKNOWN      */
    uint8_t  fallingProb;  /* P(Falling) scaled 0..255                       */
    uint8_t  valid;        /* 1 if pose is a real inference this frame       */
    uint8_t  winDown;      /* 1 if window leg says sustained down-state      */
    int16_t  winHsCm;      /* 2nd-highest point's height above floor, cm     */
    uint8_t  winLowRun;    /* consecutive low frames (saturates at 255)      */
    uint8_t  winValid;     /* 1 if the window leg had >=2 points this frame  */
} PoseResult;

/* One tracked target's raw kinematics for a frame, in the 6844 Z-up frame.
 * posZ is the RAW track height in metres (radar frame); the z-offset remap to
 * TI's reference is applied inside pose_mlp using poseZOffset. */
typedef struct PoseTrackKin_t
{
    uint32_t tid;
    float    posX, posY, posZ;
    float    velY, velZ;
    float    accY, accZ;
} PoseTrackKin;

/* One Cartesian detection point (major-motion set), 6844 Z-up frame. Used by
 * the host tests and as a convenience buffer; the firmware does NOT copy into
 * this -- it reads its own dpcAoAObjOutCartExt in place via PosePointGet. */
typedef struct PosePoint_t
{
    float x, y, z;
    float snr;             /* linear or dB -- only relative height ordering
                              and the trained snr scale matter               */
} PosePoint;

/*!
 * @brief  Read point i's Cartesian coords + snr from the caller's own buffer.
 *
 * Lets PoseMlp_process gate points without a copy: the firmware passes
 * dpcAoAObjOutCartExt (float x/y/z + int16 snr) directly and scales snr in the
 * accessor, avoiding a ~32 KB PosePoint scratch array. ctx is the caller's
 * point array base; i is 0..numPoints-1.
 */
typedef void (*PosePointGet)(const void *ctx, uint32_t i,
                             float *x, float *y, float *z, float *snr);

/*!
 * @brief  Reset all per-track ring buffers. Call once at config time.
 */
void PoseMlp_init(void);

/*!
 * @brief  Set the height remap so a standing person reads TI's reference posz.
 *         feature_posz = raw_posZ + zOffset. Field-calibrated via CLI poseCfg.
 */
void PoseMlp_setZOffset(float zOffset);

/*!
 * @brief  Configure the window (sustained-down) leg's floor geometry + timing.
 *
 * A point's height above the floor is computed in the SAME world transform the
 * server uses (points are radar-frame; world2sensor is unused in this demo):
 *     h = mountM + z*cos(tiltRad) - y*sin(tiltRad)
 * A track is "down" when its 2nd-highest point's h stays <= marginM for
 * `sustain` frames, and clears after `clear` frames above. Defaults mirror
 * pc/falldet/window.py (margin 0.45 m, sustain/clear 5).
 */
void PoseMlp_setWindowCfg(float mountM, float tiltRad, float marginM,
                          uint8_t sustain, uint8_t clear);

/*!
 * @brief  Classify every target for this frame.
 *
 * For each target: gate the point set to that target, build the 20-feature
 * frame, push it into the track's ring buffer, and -- once 8 frames are
 * buffered -- run inference. Tracks absent this frame are aged out.
 *
 * @param kin       array of per-target kinematics (numTargets entries)
 * @param numTargets number of targets
 * @param ptsCtx    opaque base of the caller's point buffer (read in place)
 * @param numPoints number of points
 * @param getPoint  reads point i's x/y/z/snr from ptsCtx (no copy; firmware
 *                  passes dpcAoAObjOutCartExt directly, host tests a PosePoint[])
 * @param out       filled with up to numTargets PoseResult (caller-sized >= numTargets)
 * @return          number of PoseResult written (== numTargets, capped at POSE_MAX_TRACKS)
 */
uint32_t PoseMlp_process(const PoseTrackKin *kin, uint32_t numTargets,
                         const void *ptsCtx, uint32_t numPoints,
                         PosePointGet getPoint, PoseResult *out);

#endif /* POSE_MLP_H_ */
