/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MMWAVE_DEMO_H
#define MMWAVE_DEMO_H

#include <drivers/uart.h>
#include <drivers/adcbuf.h>
#include <drivers/cbuff.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HeapP.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kernel/dpl/ClockP.h>


#include <common/mmwave_error.h>
#include <common/syscommon.h>
#include <control/mmwave/mmwave.h>
#include <datapath/dpif/dpif_pointcloud.h>

#include <common_mss_dss/msg_ipc/msg_ipc.h>
#include <source/mmw_cli.h>
#include <source/lvds_streaming/mmw_lvds_stream.h>
#include <source/pose/pose_mlp.h>          /* Spatial3D: per-track pose MLP */
#include <datapath/dpu/rangeproc/v1/rangeprochwa.h>
#include <datapath/dpu/cfarproc/v1/cfarprochwa.h>
#include <datapath/dpu/dopplerproc/v1/dopplerprochwa.h>
#include <datapath/dpu/aoa2dproc/v1/aoa2dproc.h>
#include <datapath/dpu/trackerproc/v0/trackerproc.h>

#include <snr3dhmproc/snr3dhmproc.h>

#include <source/calibrations/factory_cal.h>
#include <common_mss_dss/dpif_mss_dss.h>


#ifdef __cplusplus
extern "C" {
#endif

/** @brief Output Point cloud list size in number of list elements */
#define MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE DPIF_DOA_OUTPUT_MAXPOINTS

/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32


/*! @brief CFAR threshold encoding factor
 */
#define MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)

/*! @brief Demo freeRTOS tasks priorities
 */
#define DPC_TASK_PRI (5U)
#define ADC_FILEREAD_TASK_PRI (4U)
#define TLV_TASK_PRI (3U)
#define CLASSIFIER_TASK_PRIORITY (2U)
#define CLI_TASK_PRIORITY (1U)

/*! @brief Demo freeRTOS tasks stack sizes
 */
#define DPC_TASK_STACK_SIZE 8192
#define ADC_FILEREAD_TASK_STACK_SIZE 1024
#define TLV_TASK_STACK_SIZE 2048
#define CLASSIFIER_TASK_STACK_SIZE 7*1024 //ToDo Tweak this

/*! MSS L3 RAM buffer size for object detection DPC */
#define MSS_L3_MEM_SIZE (0xB0000 + 0x70000)

/*! MSS Local RAM buffer size for object detection DPC */
#define MSS_CORE_LOCAL_MEM_SIZE ((8U+6U+4U+2U+8U) * 1024U) //TODO: Update

/*! MSS Local RAM buffer size for ID/SBR/CPD */
#define MSS_CORE_LOCAL_MEM2_SIZE  (150*1024u)

#define DPC_ADC_FILENAME_MAX_LEN 256

/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/**
 * @brief   Error Code: Out of L3 RAM during radar cube allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE            (DP_ERRNO_OBJECTDETECTION_BASE-1)

/**
 * @brief   Error Code: Out of L3 RAM during detection matrix allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX            (DP_ERRNO_OBJECTDETECTION_BASE-2)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing range DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-3)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing doppler DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOA_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-4)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing doppler DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-5)

/**
 * @brief   Error Code: Out of Core Local RAM for range profile
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_PROFILE    (DP_ERRNO_OBJECTDETECTION_BASE-6)

/**
 * @brief   Error Code: Out of L3 RAM during ADC test buffer allocation allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_ADC_TEST_BUFF            (DP_ERRNO_OBJECTDETECTION_BASE-7)

/**
 * @brief   Error Code: Invalid configuration
 */
#define DPC_OBJECTDETECTION_EINVAL_CFG                              (DP_ERRNO_OBJECTDETECTION_BASE-8)

/**
 * @brief   Error Code: Antenna geometry configuration failed
 */
#define DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED            (DP_ERRNO_OBJECTDETECTION_BASE-9)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing doppler DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA2D_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-10)

/**
 * @brief   Error Code: Out of Core Local RAM.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_OUT                  (DP_ERRNO_OBJECTDETECTION_BASE-11)
/**
 * @brief   Error Code: Out of Core Local RAM.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_OUT_SIDE_INFO        (DP_ERRNO_OBJECTDETECTION_BASE-12)
/**
 * @brief   Error Code: Out of Core Local RAM for CFAR Doppler detection output
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_CFAR_DOPPLER_DET_OUT_BIT_MASK    (DP_ERRNO_OBJECTDETECTION_BASE-13)
/**
 * @brief   Error Code: Out of Core Local RAM for CFAR Doppler detection output
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_CFAR_OUT_DET_LIST                (DP_ERRNO_OBJECTDETECTION_BASE-14)

/**
 * @brief   Error Code: Out of Core Local RAM for detection Azimuth index output
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_2_AZIM_IDX           (DP_ERRNO_OBJECTDETECTION_BASE-15)

/**
 * @brief   Error Code: Out of Core Local RAM for detection Elevation angle output
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_ELEVATION_ANGLE      (DP_ERRNO_OBJECTDETECTION_BASE-16)

/**
 * @brief   Error Code: Out of Core Local RAM for AOA Scratch buffer
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_SCRATCH_BUFFER               (DP_ERRNO_OBJECTDETECTION_BASE-17)

/**
 * @brief   Error Code: Out of memory for HWA save location for range DPU
 */
#define DPC_OBJECTDETECTION_ENOMEM__RANGE_HWA_PARAM_SAVE_LOC          (DP_ERRNO_OBJECTDETECTION_BASE-18)

/**
 * @brief   Error Code: Out of HWA Window RAM
 */
#define DPC_OBJECTDETECTION_ENOMEM__HWA_WINDOW_RAM_INTERNAL           (DP_ERRNO_OBJDETRANGEHWA_BASE - 19)

/**
 * @brief   Error Code: Out of memory for HWA save location for doa3d DPU
 */
#define DPC_OBJECTDETECTION_ENOMEM__DOA3D_HWA_PARAM_SAVE_LOC          (DP_ERRNO_OBJECTDETECTION_BASE - 20)

/**
 * @brief   Error Code: Out of memory for snr3d DPU
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_SNR3D_MATRIX               (DP_ERRNO_OBJECTDETECTION_BASE - 21)

/**
 * @brief   Error Code: Out of memory for HWA save location for snr3d DPU
 */
#define DPC_OBJECTDETECTION_ENOMEM__SNR3D_HWA_PARAM_SAVE_LOC          (DP_ERRNO_OBJECTDETECTION_BASE - 22)

/**
 * @brief   Error Code: Radar cube format not supported
 */
#define DPC_OBJECTDETECTION_ERADAR_CUBE_FORMAT_NOT_SUPPORTED          (DP_ERRNO_OBJECTDETECTION_BASE - 23)

/**
 * @brief   Error Code: Out of memory for point cloud list for feature extraction
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_POINTT_CLOUD_TO_FEATURE_EXTR   (DP_ERRNO_OBJECTDETECTION_BASE - 24)

/**
 * @brief   Error Code: Out of memory for DSP configuration
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DSP_CFG (DP_ERRNO_OBJECTDETECTION_BASE - 25)

/**
 * @brief   Error Code: Out of memory
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE_DC               (DP_ERRNO_OBJECTDETECTION_BASE - 26)

/**
 * @brief   Error Code: Out of memory for HWA save location for range DPU
 */
#define DPC_OBJECTDETECTION_ENOMEM__MACRODOPPLER_HWA_PARAM_SAVE_LOC          (DP_ERRNO_OBJECTDETECTION_BASE-27)

/**
 * @brief   Error Code: Out of L3 memory
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM               (DP_ERRNO_OBJECTDETECTION_BASE - 28)

/**
 * @brief   Error Code: steering vectors configuration error
 */
#define DPC_OBJECTDETECTION_ENUM_STEERING_VECTORS_NOT_EVEN_NUMBER        (DP_ERRNO_OBJECTDETECTION_BASE - 29)

/**
 * @brief   Error Code: steering vectors configuration error
 */
#define DPC_OBJECTDETECTION_ENUM_STEERING_VECTORS_INVAL_CFG        (DP_ERRNO_OBJECTDETECTION_BASE - 30)

/**
 * @brief   Error Code: configuring continuous bursting mode failed
 */
#define DPC_OBJECTDETECTION_ERROR_PROLONGED_BURSTING_MODE_CFG        (DP_ERRNO_OBJECTDETECTION_BASE - 31)

/**
 * @brief   Error Code: configuring EDMA for timer driven RF/DPC architecture failed
 */
#define DPC_OBJECTDETECTION_ERROR_TIMER_DRIVEN_DPC_ARCH_CFG        (DP_ERRNO_OBJECTDETECTION_BASE - 32)

#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOPPLER_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE - 33)


/**
 * @brief   Ticks to USec conversion factor for Slow clock (32.768KHZ)
 */
#define M_TICKS_TO_USEC_SLOWCLK               (30.52)


extern unsigned long long    ll_LPmode_LatencyStart;

extern unsigned long long    ll_LPmode_LatencyEnd;

/*
 * @brief Structure for parameters for timer driven RF/DPC architecture
 *
 */
typedef struct DPC_TimerDrivenArchObj_t
{
    EDMA_Handle edmaHandle;

/**
 * @brief   EDMA selector channel with two shadow param sets
 */
    DPEDMA_2LinkChanCfg edmaFrameStart;
/**
 * @brief   Total number of input events
 */
    uint16_t numInEvents;
/**
 * @brief   For EDMA selector dummy source pointer
 */
    uint32_t *dummySrcPtr;
/**
 * @brief   For EDMA selector dummy destination pointer
 */
    uint32_t *dummyDstPtr;

/**
 * @brief   For EDMA interrupt object
 */
    Edma_IntrObject intrObj;

/**
 * @brief   For EDMA interrupt object
 */
    ClockP_Object clockObj;

    int32_t frameStartIntCounter;
    int32_t numFrames;

    /*! @brief  0-First frame ISR is not started 1-First frame ISR is started */
    uint8_t firstFrameStartIsrStarted;

} DPC_TimerDrivenArchObj;

/*
 * @brief Configuration structure for Prolonged (Continuous) bursting mode
 *
 */
typedef struct DPC_prolonedBurstingObj_t
{
    EDMA_Handle edmaHandle;

/**
 * @brief   EDMA selector channel with two shadow param sets
 */
    DPEDMA_3LinkChanCfg edmaEvtSplit;
/**
 * @brief   Total number of input events
 */
    uint16_t numInEvents;
/**
 * @brief   Number of chained events. (Number of gated events = numInEvents-numOutEvents)
 */
    uint16_t numOutEvents;

/**
 * @brief   Index of first passing through event
 */
    uint16_t startPassThroughEventIdx;

/**
 * @brief   For EDMA selector dummy source pointer
 */
    uint32_t *dummySrcPtr;
/**
 * @brief   For EDMA selector dummy destination pointer
 */
    uint32_t *dummyDstPtr;
/**
 * @brief   EDMA channel of the range DPU, to which the input events are chained to.
 */
    uint8_t edmaChainChannel;

} DPC_prolonedBurstingObj;

/*
 * @brief This is the result structure reported from DPC's registered processing function
 *        to the application through the DPM_Buffer structure. The DPM_Buffer's
 *        first fields will be populated as follows:
 *        pointer[0] = pointer to this structure.
 *        size[0] = size of this structure i.e sizeof(DPC_ObjectDetection_Result)
 *
 *        pointer[1..3] = NULL and size[1..3] = 0.
 */
typedef struct DPC_ObjectDetection_ExecuteResult_t
{
    /*! @brief      Total Number of detected objects */
    uint32_t        numObjOut;

    /*! @brief      Detected objects output list of @ref numObjOut elements */
    DPIF_PointCloudCartesian *objOut;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              of @ref numObjOut elements */
    DPIF_PointCloudSideInfo *objOutSideInfo;

    /*! @brief      Range Azimuth Heatmap [0] - for Major and [1] - for Minor Motion*/
    uint32_t        *rngAzHeatMap;

    /*! @brief      Range Doppler Heatmap */
    uint16_t        *rngDopplerHeatMap;

    DPU_TrackerProc_OutParams trackerOutParams;

} DPC_ObjectDetection_ExecuteResult;

/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path.
 */
typedef struct MmwDemo_output_message_tl_t
{
    /*! @brief   TLV type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

} MmwDemo_output_message_tl;

/*!
 * @brief
 * Structure holds the message body for the  Point Cloud units
 *
 * @details
 * Reporting units for range, azimuth, and doppler
 */
typedef struct MmwDemo_output_message_point_uint_t
{
    /*! @brief elevation  reporting unit, in radians */
    float elevationUnit;
    /*! @brief azimuth  reporting unit, in radians */
    float azimuthUnit;
    /*! @brief Doppler  reporting unit, in m/s */
    float dopplerUnit;
    /*! @brief range reporting unit, in m */
    float rangeUnit;
    /*! @brief SNR  reporting unit, linear */
    float snrUint;

} MmwDemo_output_message_point_unit;

/*!
 * @brief
 * Structure holds the message body to UART for the  Point Cloud
 *
 * @details
 * For each detected point, we report range, azimuth, and doppler
 */
typedef struct MmwDemo_output_message_UARTpoint_t
{
    /*! @brief Detected point elevation, in number of azimuthUnit */
    int8_t elevation;
    /*! @brief Detected point azimuth, in number of azimuthUnit */
    int8_t azimuth;
    /*! @brief Detected point doppler, in number of dopplerUnit */
    int16_t doppler;
    /*! @brief Detected point range, in number of rangeUnit */
    uint16_t range;
    /*! @brief Range detection SNR, in number of snrUnit */
    uint16_t snr;

} MmwDemo_output_message_UARTpoint;

typedef struct MmwDemo_output_message_point_major_uint_t
{
    /*! @brief x/y/z coordinates reporting unit, in m */
    float       xyzUnit;

    /*! @brief Doppler  reporting unit, in m/s */
    float       dopplerUnit;

    /*! @brief SNR  reporting unit, dB */
    float       snrUint;

    /*! @brief Noise reporting unit, dB */
    float       noiseUint;

    /*! @brief number of detected points */
    uint16_t  numDetectedPoints;

} MmwDemo_output_message_point_major_unit;

/*!
 * @brief
 * Structure holds the message body to UART for the  Point Cloud
 *
 * @details
 * For each detected point, we report x,y,z, Doppler, SNR and detection mode
 */
typedef struct MmwDemo_output_message_UARTpoint_major_t
{
    /*! @brief Detected point x, in number of x units */
    int16_t      x;
    /*! @brief Detected point y, in number of y units */
    int16_t      y;
    /*! @brief Detected point z, in number of z units */
    int16_t      z;
    /*! @brief Detected point doppler, in number of dopplerUnit */
    int16_t      doppler;
    /*! @brief Range detection SNR, in number of snrUnits */
    uint8_t      snr;
    /*! @brief Detected point noise value in in number of noiseUnits */
    uint8_t      noise;
} MmwDemo_output_message_UARTpoint_major;

typedef struct MmwDemo_output_message_UARTpointCloud_major_t
{
    MmwDemo_output_message_tl       header;
    MmwDemo_output_message_point_major_unit pointUint[2];
    MmwDemo_output_message_UARTpoint_major    point[MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE];
} MmwDemo_output_message_UARTpointCloud_major;


typedef struct MmwDemo_output_message_UARTpointCloud_t
{
    MmwDemo_output_message_tl       messageTL;
    MmwDemo_output_message_point_unit pointUnit;
    MmwDemo_output_message_UARTpoint    point[MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE];
} MmwDemo_output_message_UARTpointCloud;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_output_message_stats_t
{
    /*! @brief   Interframe processing time in usec */
    uint32_t     interFrameProcessingTime;

    /*! @brief   Transmission time of output detection information in usec */
    uint32_t     transmitOutputTime;

    /*! @brief   DSP processing time in usec */
    uint32_t     dspProcessingTime;

    /*! @brief   Current Power Sensor Readings for 1.8V, 3.3V, 1.2V and 1.2V RF rails respectively expressed in 100 uW (1LSB = 100 uW) */
    uint16_t     powerMeasured[4];

    /*! @brief   Temperature Readings: Rx, Tx, PM, DIG. in C degrees, 1LSB = 1 deg C, signed */
    int16_t      tempReading[4];

} MmwDemo_output_message_stats;

/*
 * @brief structure holds calibration save configuration.
 */
typedef struct MmwDemo_factoryCalibCfg_t
{
    /*! @brief      Enable/Disable calibration save process  */
    uint8_t         saveEnable;

    /*! @brief      Enable/Disable calibration restore process  */
    uint8_t         restoreEnable;

    /*! @brief      RX channels gain setting for factory calibration */
    uint8_t         rxGain;

    /*! @brief      TX channels power back-off setting for calibration (in 0.5 dB Resolution). */
    uint8_t         txBackoffSel;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t        flashOffset;

} MmwDemo_factoryCalibCfg;

typedef struct DPC_ObjectDetectionRangeHWA_ProfileTimeStamp_t
{
    uint32_t rdInd;
    uint32_t timeInUsec;
} DPC_ObjectDetectionRangeHWA_ProfileTimeStamp;

/*
 * @brief Stats structure to convey to Application timing and related information.
 */
typedef struct DPC_ObjectDetection_Stats_t
{
    /*! @brief   Counter which tracks the number of frame start interrupt */
    uint32_t      frameStartIntCounter;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStamp[4];

    /*! @brief   Frame period in usec */
    uint32_t      framePeriod_us;

    /*! @brief   Chirping time in usec */
    uint32_t      chirpingTime_us;

    /*! @brief   Total active time in usec */
    uint32_t      totalActiveTime_us;

    /*! Frame Idle Time available */
    uint64_t    ll_FrameIdleTime_us;

    /*! Low Power Mode Latency in usec */
    double   d_LPmode_Latency_us;

    /*! @brief   Inter-frame start CPU time stamp */
    uint32_t      interFrameStartTimeStamp;

    /*! @brief   Inter-frame end CPU time stamp */
    uint32_t      interFrameEndTimeStamp;

    /*! @brief   UART data transfer end CPU time stamp */
    uint32_t      uartTransferEndTimeStamp;

    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp chirpingCompletion;
    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp intrusionDetCompletion;
    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp pointCloudCompletion;
    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp featuresCompletion;
    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp predictionsCompletion;
    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp uartTransStart;
    DPC_ObjectDetectionRangeHWA_ProfileTimeStamp uartTransCompletion;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStampUs;
    /*! @brief  Frame start time using Slow CLock. This is used in low power mode */
    uint64_t frameStartTimeStampSlowClk;

} DPC_ObjectDetection_Stats;

typedef struct HwaDmaTrigChanPoolObj_t
{
    uint16_t dmaTrigSrcNextChan;
} HwaDmaTrigChanPoolObj;

typedef struct HwaWinRamMemoryPoolObj_t
{
    uint16_t memStartSampleIndex;
} HwaWinRamMemoryPoolObj;

/**
 * @brief
 *  ADC Data Sorce Cfg
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  ADC Data Source to the DPC.
 */
typedef struct ADC_Data_Source_Cfg_t
{
    /*! @brief      Source for ADC Data */
    uint8_t source;

    /*! @brief      ADC filename with full path */
    char fileName[DPC_ADC_FILENAME_MAX_LEN];

}ADC_Data_Source_Cfg;

/**
 * @brief Range Bias and rx channel gain/phase measurement configuration.
 *
 */
typedef struct DPC_ObjectDetection_MeasureRxChannelBiasCliCfg_t
{
    /*! @brief  1-enabled 0-disabled */
    uint8_t enabled;

    /*! @brief  Target distance during measurement (in meters) */
    float targetDistanceMts;

    /*! @brief  Search window size (in meters), the search is done in range
     *          [-searchWinSize/2 + targetDistance, targetDistance + searchWinSize/2] */
    float searchWinSizeMts;

} DPC_ObjectDetection_MeasureRxChannelBiasCliCfg;

/**
 * @brief
 *  Millimeter Wave Demo Gui Monitor Selection
 *
 * @details
 *  The structure contains the selection for what information is placed to
 *  the output packet, and sent out to GUI. Unless otherwise specified,
 *  if the flag is set to 1, information
 *  is sent out. If the flag is set to 0, information is not sent out.
 *
 */
typedef struct MmwDemo_GuiMonSel_t
{
    /*! @brief   if 1: Send list of detected objects (see @ref DPIF_PointCloudCartesian) and
     *                 side info (@ref DPIF_PointCloudSideInfo).\n
     *           if 2: Send list of detected objects only (no side info)\n
     *           if 0: Don't send anything */
    uint8_t      pointCloud;

    /*! @brief   Send range profile */
    uint8_t      rangeProfile;

    /*! @brief   Send noise profile */
    uint8_t      noiseProfile;

    /*! @brief   Send stats, timing information */
    uint8_t      statsInfo;

    /*! @brief   Send sensor temperature information */
    uint8_t      temperatureInfo;

    /*! @brief   Send sensor temperature information */
    uint8_t      rangeAzimuthHeatMap;

    /*! @brief   Send sensor temperature information */
    uint8_t      rangeDopplerHeatMap;

    /*! @brief   Send tracker information */
    uint8_t      trackerInfo;

    /*! @brief   Spatial3D DEPRECATED (Phase 1): these once configured the on-chip fall
     *           state machine, which was removed (the tracker freezes at fall). Cube
     *           extraction is now server-driven per range bin via `cubeQuery` (see the
     *           tbcQuery* fields in MmwDemo_MSS_MCB_t). Retained only so a `trackBinCubeCfg`
     *           line in a legacy .cfg still parses (the handler is a no-op); nothing reads
     *           these anymore. */
    uint8_t      trackBinCubeEnable;
    uint8_t      trackBinHalfWin;
    uint16_t     trackBinVelThr_mmps;
    uint16_t     trackBinZThrLow_cm;
    uint16_t     trackBinVelZdownThr_mmps;
    uint16_t     trackBinConfirmN;

} MmwDemo_GuiMonSel;

/* ---- Spatial3D: range-window zero-Doppler cube extraction (TLV 320) ----
 * Phase 1: the on-chip fall state machine (velZ/height ARM-CONFIRM-BURST) was REMOVED
 * (data-disproven: the tracker freezes at fall, so a track-gated burst never fires when
 * it matters). Extraction is now driven purely by the server's `cubeQuery` at a given
 * RANGE BIN - see MmwDemo_MSS_MCB_t.tbcQuery* and MmwDemo_tbcExtractBin(). */
#define TBC_MAX_ENTRIES   40
#define TBC_MAX_VIRT_ANT  16
typedef struct MmwDemo_TrackBinEntry_t
{
    uint32_t      tid;                    /*!< owning track id (0 = range-query, no track) */
    uint16_t      rangeBin;               /*!< range bin index */
    int16_t       velMag_mmps;            /*!< reserved (0 for range-query entries) */
    float         range_m;                /*!< rangeBin * rangeStep, metres */
    cmplx16ImRe_t vec[TBC_MAX_VIRT_ANT];  /*!< zero-Doppler coherent-mean 16-antenna vector */
} MmwDemo_TrackBinEntry;

/**
 * @brief
 *  Millimeter Wave Demo Gui Monitor Selection
 *
 * @details
 *  The structure contains the selection for what information is placed to
 *  the output packet, and sent out to GUI. Unless otherwise specified,
 *  if the flag is set to 1, information
 *  is sent out. If the flag is set to 0, information is not sent out.
 *
 */
typedef struct MmwDemo_DbgGuiMonSel_t
{

    /*! @brief   For debugging: intrusion detection  3D Detection Matrix */
    uint8_t       dbgDetMat3D;

    /*! @brief   For debugging: intrusion detection  */
    uint8_t       dbgSnr3D;

    /*! @brief   For debugging: intrusion detection  */
    uint8_t       dbgAntGeometry;

    /*! @brief   For debugging: intrusion detection  */
    uint8_t       dbgDetMatSlice;

    /*! @brief   For debugging: intrusion detection  */
    uint8_t       dbgSnrMatSlice;

    /*! @brief   For debugging: SBR/CPD  */
    uint8_t     exportCoarseHeatmap;
    uint8_t     exportRawCfarDetList;
    uint8_t     exportZoomInHeatmap;

    /*! @brief  export portion of radar cube: chirps corresponding to the current frame. For computing breathing rate SNR feature */
    uint8_t     radCubeFreshChunk;

    /*! @brief  Macro-Doppler heatmap (doppler bins x total num steering vectors (voxel points)) */
    uint8_t     macroDopplerHeatmap;

    /*! @brief  Breathing heatmap (dopplerBins (half spectrum)  x total number steering vectors (voxel points)) */
    uint8_t     breathingHeatmap;

} MmwDemo_DbgGuiMonSel;





/**
 * @brief
 *  Select chirps to be exported via UART
 *
 * @details
 *
 */
typedef struct MmwDemo_ExportRadarCubeChunkCfg_t
{

    /*! @brief   chirp start index */
    int16_t      chirpStartIdx;

    /*! @brief   chirp step index  */
    int16_t      chirpStepIdx;

    /*! @brief   NUmber of chirps  */
    int16_t      numChirps;

    /*! @brief   reserved  */
    int16_t      reserved;

} MmwDemo_ExportRadarCubeChunkCfg;


/* The structure defines a position in spherical coordinates */
typedef union
{
    float a[3];
    struct
	{
        float range;    /* Range, m */
        float azimuth;  /* Azimuth, rad */
        float elev;     /* Elevation, rad */
    };
} FEXTRACT_spherical_position;


/* The structure defines a position in cartesian coordinates */
typedef union
{
    float a[3];
    struct
	{
        float posX;     /* X dimension (left-right), m */
        float posY;     /* Y dimension (near-far), m */
        float posZ;     /* Z dimension (height), m */
    };
} FEXTRACT_cartesian_position;


/* Measurement point structure
 *  The structure describes measurement point format
 */
typedef union
{
    float a[5];
    struct
    {
        union
        {
           FEXTRACT_spherical_position vectorSph;       /* Spherical vector */
           FEXTRACT_cartesian_position vectorCart;      /* Cartesian vector */
        };
        float doppler;    /* Radial velocity, m/s */
        float snr;        /* Signal-to-noise ratio, linear */
    };
} FEXTRACT_measurementPoint;

typedef struct MmwDemo_antennaGeometryAnt_t
{

    /*! @brief  row index in steps of lambda/2 */
    int8_t row;

    /*! @brief  row index in steps of lambda/2 */
    int8_t col;

} MmwDemo_antennaGeometryAnt;

typedef struct MmwDemo_antennaGeometryCfg_t
{
    /*! @brief  Distance between antennas in x dimension (in meters) */
    float antDistanceXdimMts;
    /*! @brief  Distance between antennas in z dimension (in meters)*/
    float antDistanceZdimMts;

    /*! @brief  virtual antenna positions */
    MmwDemo_antennaGeometryAnt ant[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
} MmwDemo_antennaGeometryCfg;

typedef struct CLI_GuiMonSel_t
{
    /*! @brief   Send list of detected objects (see @ref MmwDemo_detectedObj_t) */
    uint8_t        pointCloud;

    /*! @brief   Send range profile array  */
    uint8_t        rangeProfile;

    /*! @brief   Send noise floor profile */
    uint8_t        noiseProfile;

    /*! @brief   Send complex range bins at zero doppler, all antenna symbols for range-azimuth heat map */
    uint8_t        rangeAzimuthHeatMap;

    /*! @brief   Send complex range bins at zero doppler, (all antenna symbols), for range-azimuth heat map */
    uint8_t        rangeDopplerHeatMap;

    /*! @brief   Send stats */
    uint8_t        statsInfo;

    /*! @brief   Send ADC samples of the last chirp pair in the frame */
    uint8_t       adcSamples;

    uint8_t       reserved;

} CLI_GuiMonSel;

typedef struct CLI_aoaProcCfg_t
{
    /*! @brief  Azimuth FFT size */
    uint16_t    azimuthFftSize;

    /*! @brief  Elevation FFT size */
    uint16_t    elevationFftSize;

} CLI_aoaProcCfg;

/**
* @brief
*  ADC logging configuration
*
* @details
*  The structure is used to hold all the relevant configuration
*  for the ADC logging through LVDS.
*/
typedef struct CLI_adcLoggingCfg_t
{
    /**
     * @brief  enabled/disabled flag
     */
    uint8_t enable;

} CLI_adcLoggingCfg;



/**
 * @brief Parameters for rx channel compensation procedure
 *
 */
typedef struct DPC_ObjDet_rangeBiasRxChPhaseMeasureParams_t
{

    /*! @brief  range step (meters/bin) */
    float rangeStep;

    /*! @brief  one over range step (meters/bin) */
    float oneOverRangeStep;

    /*! @brief  target distance in range bins */
    float trueBinPosition;
    
    /*! @brief  Range peak search left index  */
    int16_t   rngSearchLeftIdx;

    /*! @brief  Range peak search right index */
    int16_t   rngSearchRightIdx;

} DPC_ObjDet_rangeBiasRxChPhaseMeasureParams;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 */
typedef struct DPC_ObjDet_compRxChannelBiasFloatCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    float rxChPhaseComp[2 * SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPC_ObjDet_compRxChannelBiasFloatCfg;

typedef struct DPC_memUsage_t
{
    /*! @brief   Indicates number of bytes of L3 memory allocated to be used by DPC */
    uint32_t L3RamTotal;

    /*! @brief   Indicates number of bytes of L3 memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t L3RamUsage;

    /*! @brief   Indicates number of bytes of Core Local memory allocated to be used by DPC */
    uint32_t CoreLocalRamTotal;

    /*! @brief   Indicates number of bytes of Core Local memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t CoreLocalRamUsage;

    /*! @brief   Indicates number of bytes of system heap allocated */
    uint32_t SystemHeapTotal;

    /*! @brief   Indicates number of bytes of system heap used at the end of PreStartCfg */
    uint32_t SystemHeapUsed;

    /*! @brief   Indicates number of bytes of system heap used by DCP at the end of PreStartCfg */
    uint32_t SystemHeapDPCUsed;

    /*! @brief   Tracker heap stats */
    HeapP_MemStats trackerHeapStats;
} DPC_memUsage;

/*
 * @brief Memory Configuration used during init API
 */
typedef struct DPC_ObjectDetection_MemCfg_t
{
    /*! @brief   Start address of memory provided by the application
     *           from which DPC will allocate.
     */
    void *addr;

    /*! @brief   Size limit of memory allowed to be consumed by the DPC */
    uint32_t size;
} DPC_ObjectDetection_MemCfg;

/*
 * @brief Memory pool object to manage memory based on @ref DPC_ObjectDetection_MemCfg_t.
 */
typedef struct MemPoolObj_t
{
    /*! @brief Memory configuration */
    DPC_ObjectDetection_MemCfg cfg;

    /*! @brief   Pool running adress.*/
    uintptr_t currAddr;

    /*! @brief   Pool max address. This pool allows setting address to desired
     *           (e.g for rewinding purposes), so having a running maximum
     *           helps in finding max pool usage
     */
    uintptr_t maxCurrAddr;
} MemPoolObj;

/**
 * @brief
 *  The structure specifies common signal processing chain pearameters for all running modes.
 *
 * @details
 */
typedef struct MmwDemo_SigProcChainCommonCfg_t
{

    /**
     * @brief Frame period in usec
     */
    uint32_t framePeriodicityus;

    /**
     * @brief Number of frames per sliding window
     */
    uint16_t numFrmPerSlidingWindow;

    /**
     * @brief Number of bursts per frame used in the processing chain 
     */
    uint16_t numOfBurstsInFrame;

    /**
     * @brief Index of the starting burst that is input to processing chain
     */
    uint16_t startBurstIdx;

    /**
     * @brief Number of frames to run
     */
    uint16_t numOfFrames;



} MmwDemo_SigProcChainCommonCfg;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo.
 */
typedef struct MmwDemo_MSS_MCB_t
{

    /*! @brief      UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief      ADC buffer handle */
    ADCBuf_Handle               adcBuffHandle;

    /*! @brief   Handle of the EDMA driver, used for other edma in signal chain */
    EDMA_Handle                  edmaHandle;
    /*! @brief   Handle of the EDMA driver used for CBUFF */
    EDMA_Handle                  edmaHandle1;

    /*! @brief   Number of EDMA event Queues (tc) */
    uint8_t                     numEdmaEventQueues;

    // /*! @brief   Rf frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
    // double                      rfFreqScaleFactor;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            demoInitTaskCompleteSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            cliInitTaskCompleteSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            TestSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            tlvSemHandle;

    /*! @brief   Semaphore Object to pend in classifier task */
    SemaphoreP_Object            classifierTaskSemHandle;
    /*! @brief   Semaphore Object to pend in classifier task */
    SemaphoreP_Object            classifierTaskSem2Handle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            adcFileTaskSemHandle;

    /*! @brief   Semaphore Object  */
    SemaphoreP_Object            dpcTaskConfigDoneSemHandle;
    /*! @brief   Semaphore Object  */
    SemaphoreP_Object            uartTaskConfigDoneSemHandle;

    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

    /**
     * @brief MMWave configuration which includes frameCfg, profileTimeCfg, profileComCfg.
     */
    MMWave_Cfg                  mmWaveCfg;

    // /**
    //  * @brief Front End Run Time TX CLPC calibration command
    //  *
    //  */
    // T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD     fecTxclpcCalCmd;

    /**
     * @brief ADC read from file data structure
     *
     */
    ADC_Data_Source_Cfg              adcDataSourceCfg;

    /**
     * @brief Raw ADC data capture configuration
     *
     */
    CLI_adcLoggingCfg                adcLogging;
    
    /* @brief static clutter removal flag */
    bool                               staticClutterRemovalEnable;

    /**
     *  @brief   Range Bias and rx channel gain/phase measurement configuration
     *
     */
    DPC_ObjectDetection_MeasureRxChannelBiasCliCfg measureRxChannelBiasCliCfg;

    /**
     *  @brief   Parameters for range bias rx channel compensation procedure
     *
     */
    DPC_ObjDet_rangeBiasRxChPhaseMeasureParams measureRxChannelBiasParams;

    /**
     * @brief Rx channel compensation coefficients, specified by CLI command
     *
     */
    DPC_ObjDet_compRxChannelBiasFloatCfg compRxChannelBiasCfgMeasureOut;
    // Pedrhom, minor uses below
    //DPC_ObjDet_compRxChannelBiasFloatCfg compRxChannelBiasCfgMeasureOut;

     /* @brief Rx channel compensation coefficients, specified by CLI command
     *
     */
    DPU_AoAProc_compRxChannelBiasCfg    compRxChannelBiasCfg;

    DPU_AoAProc_compRxChannelBiasCfg    compRxChannelBiasCfgMajor;

    // /*! @brief  Configuration to setup DFP */
    // MMWave_CtrlCfg                      mmwCtrlCfg;

    /*! @brief  Configuration to open DFP */
    MMWave_OpenCfg                      mmwOpenCfg;

    /*! @brief  Rx Channel offsets in ADC buffer - not used */
    uint16_t                            rxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

    /*! @brief  Range processing DPU handle */
    DPU_RangeProcHWA_Handle             rangeProcDpuHandle;

    /* DPC configs from CLI */
    /*! @brief Number of enabled Tx antennas */
    uint16_t                            numTxAntennas;
    uint8_t                             txAntOrder[SYS_COMMON_NUM_TX_ANTENNAS];

    /*! @brief Number of enabled Rx antennas */
    uint16_t                            numRxAntennas;
    
    uint8_t                             rxAntOrder[SYS_COMMON_NUM_RX_CHANNEL];
    
    /*! @brief ADC sampling rate in in usec */
    float                               adcStartTime;

    /*! @brief ADC sampling rate in MHz */
    float                               adcSamplingRate;

    /*! @brief Number of range bins */
    uint32_t                            numRangeBins;
    /*! @brief Number of range bins */
    uint32_t                            rangeFftSize;

    /*! @brief Number of Doppler bins */
    uint32_t                            numDopplerBins;
    /*! @brief Number of Doppler chirps */
    uint32_t                            numDopplerChirps;

    // /*! @brief  Angle dimension:
    //  * 0-1Tx antenna-1Rx antenna,
    //  * 1-virtual antenna array only in azimuth dimension,
    //  * 2-two-dimensional virtual antenna array */
    // uint8_t                             angleDimension;

    MmwDemo_antennaGeometryCfg          antennaGeometryCfg;
    MmwDemo_antennaGeometryCfg          activeAntennaGeometryCfg;
    uint16_t                            numAntRow;
    uint16_t                            numAntCol;
    float                               lambdaOverDistX;
    float                               lambdaOverDistZ;

    /*! @brief Flag: 0-Low power mode disabled, 1-Low Power mode enabled, 2-Used for testing low power mode */
    uint8_t                             lowPowerMode;

    /*! @brief Flag to control in low power mode some configuration parts to be executed only once, and not to be repeated from frame to frame */
    uint8_t                             oneTimeConfigDone;

    /*! @brief L3 ram memory pool object */
    MemPoolObj    L3RamObj;

    /*! @brief Core Local ram memory pool object */
    MemPoolObj    CoreLocalRamObj;

    /*! @brief Memory pool object using RTOS API */
    HeapP_Object CoreLocalRtosHeapObj;

    /*! @brief Tracker memory pool object */
    HeapP_Object CoreLocalTrackerHeapObj;

    /*! @brief Memory Usage */
    DPC_memUsage    memUsage;

    /*! @brief ADC buffer allocated in testing mode only */
    uint8_t         *adcTestBuff;

    /*! @brief      Radar cube data */
    DPIF_RadarCube  radarCube[2];

    /*! @brief      Detection matrix */
    uint16_t  *detMatrix;

    /*! @brief      Steering vectors to zones */
    cmplx32ReIm_t * steerVec;

    /*! @brief      Range bins corresponding to steering vectors to zones */
    uint16_t        *rangeBinVec;

    /*! @brief      phase multi-frame FFT output - breathing heatmap */
    uint32_t      * phaseMultiFrmDoppOut;

    /*! @brief      Macro-Doppler multi-frame FFT output */
    uint32_t      * symbMultiFrmDoppOut;

    /*! @brief      Averaged Macro-Doppler multi-frame FFT output */
    uint32_t        *averageSymbMultiFrmDoppOut;


    /*! @brief      Range profiles to be exported to GUI */
    uint32_t      * rangeProfile;

    /*! @brief      Conversion of range index to meters */
    float           rangeStep;

    /*! @brief      Conversion of Doppler index to meters/sec */
    float           dopplerStep;

    /*! @brief      Corresponding to sampling interval (Hz) */
    float           bandwidth;
    /*! @brief      Corresponding to middle of the sampling interval (Hz) */
    float           centerFreq;

    /*! @brief      SNR Matrix */
    DPIF_DetMatrix  snrOutMatrix;

     /*! @brief      Doppler index matrix, type uint8, size = number range bins x number azimuth bins * number of elevation bins */
    DPIF_DetMatrix dopplerIndexMatrix;

    /*! @brief      Elevation index matrix, type uint8, size = number range bins x number azimuth bins*/
    DPIF_DetMatrix elevationIndexMatrix;

    /*! @brief Point cloud structure with int16 type coordinates sent to Host via UART. Includes TLV header and uints structure */
    MmwDemo_output_message_UARTpointCloud pointCloudToUart;
    MmwDemo_output_message_point_unit pointUnitInv;

    /*! @brief Structure with inverse unit scales for coordinate conversion from float type to int16 type */
    MmwDemo_output_message_point_unit pointCloudUintRecip;

    /*! @brief  Point cloud list generated by CFAR DPU */
    DPIF_PointCloudCartesianExt *cfarDetObjOut;

    /*! @brief  List of CFAR detected objects */
    DPIF_CFARDetList *cfarRngDopSnrList;

    /*! @brief  Point cloud list (floating point) sent over UART if enabled */
    DPIF_PointCloudCartesian *dpcObjOut;

    /*! @brief  Point cloud side information list  sent over UART if enabled */
    DPIF_PointCloudSideInfo *dpcObjSideInfo;

    /*! @brief  Point cloud list range/azimuth/elevation/doppler indices per point */
    DPIF_PointCloudRngAzimElevDopInd *dpcObjIndOut;

    /*! @brief  DPC stats structure */
    DPC_ObjectDetection_Stats stats;

    /*! @brief      DPC reported output stats structure */
    MmwDemo_output_message_stats outStats;

    /*! @brief Token is checked in the frame start ISR, asserted to have zero value, and incremented. At the end of UART task, it is decremented */
    uint32_t interSubFrameProcToken;
    /*! @brief Counts frames not completed in time */
    uint32_t interSubFrameProcOverflowCntr;

    /*! @brief  This is used in testing the low power mode */
    //uint64_t frameStartTimeStampSlowClk;
    
    /*! @brief  Factory calibration cofiguration for save/restore */
    MmwDemo_factoryCalibCfg    factoryCalCfg;

    /*! @brief HWA DMA trigger source channel pool */
    HwaDmaTrigChanPoolObj  HwaDmaChanPoolObj;

    /*! @brief HWA Window RAM memory pool */
    HwaWinRamMemoryPoolObj  HwaWinRamMemoryPoolObj;

    /*! @brief Number of used HWA param sets */
    uint8_t numUsedHwaParamSets;

    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    //  INTRUDER+SBR+CPD:
    MmwDemo_SigProcChainCommonCfg sigProcChainCommonCfg;
    /**
     * @brief Gui Monitor Sel
     *
     */
    MmwDemo_GuiMonSel guiMonSel;
    MmwDemo_DbgGuiMonSel dbgGuiMonSel;
    DPU_SNR3DProc_CfarCfg snr3dCfarCfg;
    DPU_SNR3DProc_CfarScndPassCfg   snr3dCfarScndPassCfg;

    /*! @brief   Bit mask of enabled rx channels */
    uint16_t cliRxEnbl;
    /*! @brief   Bit mask of enabled tx channels */
    uint16_t cliTxEnbl;


    MsgIpc_CtrlObj msgIpcCtrlObj;

    SemaphoreP_Object dspCfgDoneSemaphore;


    /* SBR/CPD - Capon related ... */
    /*! @brief DSP Pre start configuration */
    DPIF_MSS_DSS_PreStartCfg dspPreStartCfgLocal;
    /*! @brief DSP Pre start configuration in the shared memory (L3)*/
    DPIF_MSS_DSS_PreStartCfg *dspPreStartCfgShare;

    float dynamicSideLobeThr; /**< CFAR sidelobe threshold for dynamic scene. */
    float staticSideLobeThr; /**< CFAR sidelobe threshold for static scene. */


    DPIF_MSS_DSS_radarProcessOutput      *outputFromDSP;
    FEXTRACT_measurementPoint            *pointCloudToFeatExtr;

    uint16_t numDetectedPointsMajor;
    uint16_t numDetectedPointsMinor;

    /*! @brief  frame counter within sliding window */
    uint16_t frmCntrInSlidingWindowInitVal;

    /*! @brief  frame counter within sliding window used for the transfer of chunks of radar cube to host */
    uint16_t frmCntrInSlidingWindowUart;

    /*! @brief  Configuration for prolonged bursting mode */
    DPC_prolonedBurstingObj prolongedBurstingObj;

    /*! @brief  Parameters for timer driven RF/DPC architecture */
    DPC_TimerDrivenArchObj timerDrivenArchObj;

    /*! @brief  0-standard architecture  1-timer driven RF/DPC architecture*/
    uint8_t timerDrivenDpcMode;

    /*! @brief  0-normal (standard) mode (number of bursts in frame same as number of burst processed in chain), 1-prolonged (continuous) bursting mode */
    uint8_t prolongedBurstingMode;

 
    /*! @brief   this structure is used to hold all the relevant information
        for the mmw demo LVDS stream*/
    MmwDemo_LVDSStream_MCB_t    lvdsStream;

    // 3DPT Start

    uint32_t frmCntrModNumFramesPerMinorMot;
    
    uint8_t    isBpmEnabled;

    /*! @brief  Doppler processing DPU handle */
    DPU_DopplerProcHWA_Handle           dopplerProcDpuHandle;

    /*! @brief  CFAR DPU handle */
    DPU_CFARProcHWA_Handle              cfarProcDpuHandle;

    /*! @brief  AOA2D DPU handle */
    DPU_AoAProcHWA_Handle                aoa2dProcDpuHandle;

    DPU_TrackerProc_Handle trackerProcDpuHandle;

    DPU_CFARProc_CfarCfg             cfarRangeCfg;

    DPU_AoAProc_FovAoaCfg          fovAoaCfg;

    /**
     * @brief Multi object beamforming Cfg
     *
     */
    DPU_AoAProc_MultiObjBeamFormingCfg multiObjBeamFormingCfg;
    
    /*! @brief  Output Point cloud list from AoA DPU */
    DPIF_PointCloudCartesian *dpcAoAObjOut;

    /*! @brief  Output Point cloud side information list from AoA DPU */
    DPIF_PointCloudSideInfo *dpcAoAObjSideInfo;

    /*! @brief      Pointers to DPC output data */
    DPC_ObjectDetection_ExecuteResult dpcResult;
    
    /**
     * @brief Cfar Cfg - Doppler Direction
     *
     */
    DPU_CFARProc_CfarCfg             cfarDopplerCfg;

    /**
    * @brief Scenary params - sensor position, boundary of interest
    *
    */
    trackerProc_SceneryParams sceneryParams;

    /**
     * @brief CFAR field of view configuration in range domain 
     */
    DPU_CFARProc_FovCfg fovRange;

    /**
     * @brief CFAR field of view configuration in Doppler domain 
     */
    DPU_CFARProc_FovCfg fovDoppler;

    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_aoaProcCfg                   aoaProcCfg;

    uint8_t frameCount;

    uint8_t numFramesPerMinorMode;

    int32_t log2numDopplerChirps;

    DPIF_PointCloudCartesianExt *dpcAoAObjOutCartExt;

    /*! @brief   Tracker DPU Static Configuration */
    DPU_TrackerProc_Config trackerCfg;

    DPIF_PointCloudSpherical pointCloudToTrackerMinorFloat;

    DPIF_PointCloudSideInfo  sideInfoToTrackerMinor;

    /* Spatial3D: per-bin zero-Doppler cube extraction buffer. Filled in dpc_mss.c
     * (range proc done, radarCube[0] still valid), drained by the TLV transmit task. */
    uint16_t              tbcNumEntries;
    uint16_t              tbcNumVirtAnt;
    MmwDemo_TrackBinEntry tbcEntries[TBC_MAX_ENTRIES];

    /* Spatial3D: server-driven range-window cube query (track-INDEPENDENT). Armed by
     * the `cubeQuery <range_bin> <half_win> <n_frames>` CLI command; DPC_Execute then
     * extracts bin +- halfWin from radarCube[0] and emits TLV 320 for framesLeft frames,
     * then disarms. Recognises a RANGE BIN, not a track -> a lost or frozen track (as
     * happens during a fall) never blocks the fall second-check. */
    volatile uint8_t      tbcQueryActive;      /*!< 1 while a cubeQuery burst is running */
    volatile uint16_t     tbcQueryBin;         /*!< center range bin of the query window */
    volatile uint16_t     tbcQueryHalfWin;     /*!< +- range bins around the center bin  */
    volatile int32_t      tbcQueryFramesLeft;  /*!< frames still to emit for this query  */

    /* Spatial3D: per-track pose classification (auxiliary fall leg). Filled in
     * dpc_mss.c right after DPU_TrackerProc_process (track kinematics + gated
     * Cartesian points), drained into TLV 321 by the transmit task. The MLP is
     * a "free pose + fall-motion trigger" leg; the primary fall decision stays
     * server-side (window + cube-RR). See pose/pose_mlp.h. */
    uint8_t               poseEnable;          /*!< poseCfg: 1 run + emit TLV 321, 0 off */
    uint16_t              poseNumResults;      /*!< pose results valid this frame        */
    PoseResult            poseResults[POSE_MAX_TRACKS];

} MmwDemo_MSS_MCB;

#define MMWDEMO_OUTPUT_ALL_MSG_MAX 14 //Spatial3D: +TLV 320 (cube) +321 (pose)

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, and also for Mailbox communication
 *  between MSS and DSS on the XWR18xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum mmwLab_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,
    
    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE = 2,

    /*! @brief   Target List - Array of detected targets (position, velocity, error covariance) */
    MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST = 308,

    /*! @brief   Target List - Array of target indices */
    MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX = 309,

    /*! @brief   Spatial3D: per-track per-bin zero-Doppler 16-antenna cube vectors */
    MMWDEMO_OUTPUT_EXT_MSG_TRACK_BIN_CUBE = 320,

    /*! @brief   Spatial3D: per-track pose classification (Stood/Sat/Lying/Falling) */
    MMWDEMO_OUTPUT_EXT_MSG_POSE = 321,

    /*! @brief   Point Cloud - Array of detected points (range/angle/doppler) */
    MMWDEMO_OUTPUT_MSG_POINT_CLOUD = 3001,
    /*! @brief   SBR/CPD features */
    MMWDEMO_OUTPUT_MSG_OCCUPANCY_FEATURES = 3002,
    /*! @brief   Occupancy classification result */
    MMWDEMO_OUTPUT_MSG_OCCUPANCY_CLASSIFICATION_RES = 1041,
    /*! @brief   Occupancy height result */
    MMWDEMO_OUTPUT_MSG_OCCUPANCY_HEIGHT_RES = 1042,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS = 6,

    MMWDEMO_OUTPUT_MSG_INTRUSION_DET_INFO = 12,
    MMWDEMO_OUTPUT_MSG_INTRUSION_DET_3D_DET_MAT,
    MMWDEMO_OUTPUT_MSG_INTRUSION_DET_3D_SNR,

    /*! @brief   Rx Channel compensation info */
    MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO = 318,

    MMWDEMO_OUTPUT_DEBUG_ANT_GEOMETRY = 2000,
    MMWDEMO_OUTPUT_DEBUG_DET_MAT_ANGLE_SLICE = 2001,
    MMWDEMO_OUTPUT_DEBUG_SNR_MAT_ANGLE_SLICE = 2002,

    MMWDEMO_OUTPUT_DEBUG_CAPON_HEATMAP = 2003,
    MMWDEMO_OUTPUT_DEBUG_CAPON_RAW_CFAR_POINT_CLOUD = 2004,
    MMWDEMO_OUTPUT_DEBUG_CAPON_ZOOM_IN_HEATMAP = 2005,

    MMWDEMO_OUTPUT_DEBUG_RADAR_CUBE_FRESH_CHUNK = 2007,

    MMWDEMO_OUTPUT_DEBUG_AVERAGED_MACRO_DOPPLER_FFT = 2008,
    MMWDEMO_OUTPUT_DEBUG_MACRO_DOPPLER_FFT_VOXEL_HEATMAP = 2009,
    MMWDEMO_OUTPUT_DEBUG_PHASE_DOPPLER_FFT_VOXEL_HEATMAP = 2010,

    MMWDEMO_OUTPUT_MSG_MAX = 13
} mmwLab_output_message_type;

#define MMWDEMO_OUTPUT_ALL_MSG_MAX 14 //Spatial3D: +TLV 320 (cube) +321 (pose)

/*!
 * @brief
 *  Message header for reporting detection information from data path.
 *
 * @details
 *  The structure defines the message header.
 */
typedef struct MmwDemo_output_message_header_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
    uint16_t    magicWord[4];

    /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
    uint32_t     version;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;

    /*! @brief   platform type */
    uint32_t    platform;

    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   Time in CPU cycles when the message was created. For XWR16xx/XWR18xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
    uint32_t    timeCpuCycles;

    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObj;

    /*! @brief   Number of TLVs */
    uint32_t    numTLVs;

    uint32_t    subFrameNumber;

} MmwDemo_output_message_header;

typedef struct MmwDemo_output_message_headerID_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
    uint16_t    magicWord[4];

    /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
    uint32_t     version;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;

    /*! @brief   platform type */
    uint32_t    platform;

    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   Time in CPU cycles when the message was created. For XWR16xx/XWR18xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
    uint32_t    timeCpuCycles;

    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObjMajor;

    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObjMinor;

    /*! @brief   Number of TLVs */
    uint32_t    numTLVs;

    /*! @brief   For Advanced Frame config, this is the sub-frame number in the range
     * 0 to (number of subframes - 1). For frame config (not advanced), this is always
     * set to 0. */
    uint32_t    subFrameNumber;
} MmwDemo_output_message_headerID;

/* Debug Functions */
extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_debugAssert(expression) {                                      \
                                         _MmwDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }


#ifdef __cplusplus
}
#endif

#endif /* MMWAVE_DEMO_H */
