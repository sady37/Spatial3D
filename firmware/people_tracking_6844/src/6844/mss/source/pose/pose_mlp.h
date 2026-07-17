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

/* One track's pose result for the frame. */
typedef struct PoseResult_t
{
    uint32_t tid;          /* track id this result belongs to               */
    uint8_t  pose;         /* POSE_STOOD..POSE_FALLING, or POSE_UNKNOWN      */
    uint8_t  fallingProb;  /* P(Falling) scaled 0..255                       */
    uint8_t  valid;        /* 1 if pose is a real inference this frame       */
    uint8_t  pad;
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

/* One Cartesian detection point (major-motion set), 6844 Z-up frame. */
typedef struct PosePoint_t
{
    float x, y, z;
    float snr;             /* linear or dB -- only relative height ordering
                              and the trained snr scale matter               */
} PosePoint;

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
 * @brief  Classify every target for this frame.
 *
 * For each target: gate the point set to that target, build the 20-feature
 * frame, push it into the track's ring buffer, and -- once 8 frames are
 * buffered -- run inference. Tracks absent this frame are aged out.
 *
 * @param kin       array of per-target kinematics (numTargets entries)
 * @param numTargets number of targets
 * @param pts       Cartesian detection points for the frame
 * @param numPoints number of points
 * @param out       filled with up to numTargets PoseResult (caller-sized >= numTargets)
 * @return          number of PoseResult written (== numTargets, capped at POSE_MAX_TRACKS)
 */
uint32_t PoseMlp_process(const PoseTrackKin *kin, uint32_t numTargets,
                         const PosePoint *pts, uint32_t numPoints,
                         PoseResult *out);

#endif /* POSE_MLP_H_ */
