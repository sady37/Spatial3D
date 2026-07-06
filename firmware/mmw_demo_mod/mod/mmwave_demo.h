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

#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HeapP.h>

#include <drivers/uart.h>
#include <drivers/adcbuf.h>
#include <drivers/cbuff.h>
#include <control/mmwave/mmwave.h>

#include <source/mmw_cli.h>
#include <source/lvds_streaming/mmw_lvds_stream.h>

#include "FreeRTOS.h"
#include "task.h"

#include <datapath/dpu/rangeproc/v1/rangeprochwa.h>
#include <datapath/dpu/cfarproc/v1/cfarprochwa.h>
#include <datapath/dpu/dopplerproc/v1/dopplerprochwa.h>
#include <datapath/dpu/aoa2dproc/v1/aoa2dproc.h>
#include <datapath/dpif/dpif_pointcloud.h>
#include <datapath/dpif/dpif_chcomp.h>

#include <common/mmwave_error.h>
#include <common/syscommon.h>


#ifdef __cplusplus
extern "C" {
#endif


#define SPI_ADC_DATA_STREAMING (1U)

/* Max Frame Size for FTDI chip is 64KB */
#define MAXSPISIZEFTDI               (65536U)

/*! @brief ADC Data Logging macro */
#define ADC_DATA_LOGGING_DISABLE              (0U)
#define ADC_DATA_LOGGING_LVDS_STREAMING       (1U)
#define ADC_DATA_LOGGING_SPI_STREAMING        (2U)

/** @brief Output Point cloud list size in number of list elements */
#define MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE 500

/*! @brief CFAR threshold encoding factor
 */
#define MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)

/*! @brief Demo freeRTOS tasks priorities
 */
#define DPC_TASK_PRI (5U)
#define ADC_FILEREAD_TASK_PRI (4U)
#define TLV_TASK_PRI (3U)
#define CLI_TASK_PRIORITY (1U)

/*! @brief Demo freeRTOS tasks stack sizes
 */
#define DPC_TASK_STACK_SIZE 8192
#define ADC_FILEREAD_TASK_STACK_SIZE 4096
#define TLV_TASK_STACK_SIZE 2048

#define DPC_ADC_FILENAME_MAX_LEN 256

/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/* 
 * LSB in "DSS_CTRL::ADCBUFCFG1_EXTD_ADCBUFINTGENDLY" = 1 DSS clock cycle. Whatever the value is configured, 2 cycle delay will be additionally seen due to Pipeline delay. 
 * For eg: When value 1 is configured, effective delay is 1*(5ns) + 2*(5ns) = 15nsec.
 * NOTE: These numbers are with assumption that DSS CLock is running at 200MHz 
 * */
/* MinDelay = 1.1us = 218 LSBs, MaxDelay = 3.6us = 718 LSBs */   
#define MIN_ADC_PER_CHIRP_DELAY 218U
/* ADC per chirp dither range is  718-218 = 500 LSBs */
#define ADC_PER_CHIRP_DITHER_RANGE 500U

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
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOPPLER_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-4)

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
 * @brief   Ticks to USec conversion factor for Slow clock (32.768KHZ)
 */
#define M_TICKS_TO_USEC_SLOWCLK               (30.52)

/**
 * @brief   Maximum GPADC channels supported
 */
#define MAX_GPADC_CHANNELS   (4U)

/* Size of Buffer to store ADC Data */
/*Important Note: User has to modify this size based on amount of ADC data streamed per frame. Currently buffer is configured for 128KB */
#define ADC_DATA_BUFF_MAX_SIZE (131072U)
#define EDMA_TEST_EVT_QUEUE_NO      (0U)

extern uint8_t adcbuffer[ADC_DATA_BUFF_MAX_SIZE];
extern uint32_t adcDataPerFrame;



extern unsigned long long    ll_LPmode_LatencyStart;

extern unsigned long long    ll_LPmode_LatencyEnd;
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

    /*! @brief      Range Azimuth Heatmap*/
    uint32_t        *rngAzHeatMap;

    /*! @brief      Range Doppler Heatmap */
    uint16_t        *rngDopplerHeatMap;

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

} MmwDemo_output_message_point_unit;

/*!
 * @brief
 * Structure holds the message body to UART for the  Point Cloud
 *
 * @details
 * For each detected point, we report x,y,z, Doppler, SNR and detection mode
 */
typedef struct MmwDemo_output_message_UARTpoint_t
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
} MmwDemo_output_message_UARTpoint;

typedef struct MmwDemo_output_message_UARTpointCloud_t
{
    MmwDemo_output_message_tl       header;
    MmwDemo_output_message_point_unit pointUint[2];
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
    uint32_t     interFrameProcessingTimeUs;

    /*! @brief   Transmission time of output detection information in usec */
    uint32_t     transmitOutputTimeUs;

    /*! @brief   Current Power Sensor Readings for 1.8V, 3.3V, 1.2V and 1.2V RF rails respectively expressed in 100 uW (1LSB = 100 uW) */
    uint16_t     powerMeasured[4];

    /*! 
      * @brief   Temperature Readings in degrees Celsius, with a resolution of 1 degree per unit, for the following components:
      * tempReading[0]: Average Rx temperature
      * tempReading[1]: Average Tx temperature
      * tempReading[2]: PM temperature
      * tempReading[3]: DIG temperature
      */
    int16_t      tempReading[4];

} MmwDemo_output_message_stats;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibData_t
{
    /*! @brief      Magic word for calibration data */
    uint32_t        magic;

    /*! @brief      RX TX Calibration data */
    T_RL_API_FECSS_FACT_CAL_DATA  calibData;
    
} MmwDemo_calibData;

/*
 * @brief Stats structure to convey to Application timing and related information.
 */
typedef struct DPC_ObjectDetection_Stats_t
{
    /*! @brief   Counter which tracks the number of frame start interrupt */
    uint32_t      frameStartIntCounter;

    /*! @brief   Frame period in usec */
    uint32_t      measuredframePeriodUs;

    /*! @brief   Chirping time in usec */
    uint32_t      chirpingTimeUs;

    /*! @brief   Total active time in usec */
    uint32_t      totalActiveTimeUs;

    /*! Frame Idle Time available */
    unsigned long long    ll_FrameIdleTimeus;

    /*! Low Power Mode Latency in usec */
    double   d_LPmode_Latencyus;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStampUs;

    /*! @brief   Inter-frame start CPU time stamp */
    uint32_t      ProcessingStartTimeStampUs;

    /*! @brief   Inter-frame end CPU time stamp */
    uint32_t      ProcessingEndTimeStampUs;

    /*! @brief   UART data transfer end CPU time stamp */
    uint32_t      uartTransferEndTimeStampUs;

    /*! @brief  Frame start time using Slow CLock. This is used in low power mode */
    unsigned long long frameStartTimeStampSlowClk;

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
     *          [-searchWinSizeMts/2 + targetDistanceMts, targetDistanceMts + searchWinSizeMts/2] */
    float searchWinSizeMts;

} DPC_ObjectDetection_MeasureRxChannelBiasCliCfg;

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

    uint8_t       reserved;

    /*! @brief   Spatial3D: range-antenna zero-Doppler TLV — first range bin to export */
    uint16_t       rangeAntennaStartBin;

    /*! @brief   Spatial3D: range-antenna zero-Doppler TLV — number of range bins to export
     *           (0 = disabled). Enable gate reuses @ref rangeAzimuthHeatMap. */
    uint16_t       rangeAntennaNumBins;

} CLI_GuiMonSel;

/**
 * @brief
 *  GPADC measurement configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  for the GPADC measurement configuration.
 */
typedef struct  CLI_gpAdcCfg_t
{
    /*!
     * @brief  GPADC channels ON/OFF control, 1 bit per channel. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF, GPADC channel will not be enabled for volatge measurement |
     * | Non-Zero  | ON, GPADC channel will be enabled automatically                |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]  | GPADC Channel 0 |
     * | bits [1]  | GPADC Channel 1 |
     * | bits [2]  | GPADC Channel 2 |
     * | bits [3]  | GPADC Channel 3 |
     * | bits [7:4] | Reserved |
     */
    uint8_t channelEnable;

    /*!
     * @brief  GPADC voltage prints through UART control. When enabled, the voltages are displayed through UARTB for the GPADC channels configured in @ref channelEnable  \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | Voltage prints Disable |
     * | 0x1       | Voltage prints Enable  |
     */
    uint8_t volPrintsEnable;

    /*! @brief Measured GPADC Volatage values(V) for the channels 0,1,2,3. */
    float gpAdcValVolts[MAX_GPADC_CHANNELS];
} CLI_gpAdcCfg;

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

}CLI_adcLoggingCfg;

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

    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                  edmaHandle;

    /*! @brief   Number of EDMA event Queues (tc) */
    uint8_t                     numEdmaEventQueues;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            demoInitTaskCompleteSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            cliInitTaskCompleteSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            TestSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            tlvSemHandle;

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
    
    /**
     * @brief Gui Monitor Sel
     *
     */
    CLI_GuiMonSel                    guiMonSel;

    /**
     * @brief GPADC configuration
     *
     */
    CLI_gpAdcCfg                     gpAdcCfg;         

    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_aoaProcCfg                   aoaProcCfg;

    /**
     * @brief Cfar Cfg - Range Direction
     *
     */
    DPU_CFARProc_CfarCfg             cfarRangeCfg;
    
    /**
     * @brief Cfar Cfg - Doppler Direction
     *
     */
    DPU_CFARProc_CfarCfg             cfarDopplerCfg;

    /**
     * @brief CFAR field of view configuration in range domain 
     */
    DPU_CFARProc_FovCfg fovRange;

    /**
     * @brief CFAR field of view configuration in Doppler domain 
     */
    DPU_CFARProc_FovCfg fovDoppler;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_AoAProc_FovAoaCfg          fovAoaCfg;

    /**
     * @brief Multi object beamforming Cfg
     *
     */
    DPU_AoAProc_MultiObjBeamFormingCfg multiObjBeamFormingCfg; 

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
    DPIF_compRxChannelBiasFloatCfg compRxChannelBiasCfgMeasureOut;

    /**
     * @brief Range Select Cfg
     *
     */
    DPU_CFARProc_FovCfg          rangeSelCfg;

     /* @brief Rx channel compensation coefficients, specified by CLI command
     *
     */
    DPU_AoAProc_compRxChannelBiasCfg    compRxChannelBiasCfg;

    /*! @brief  Configuration to open DFP */
    MMWave_OpenCfg                      mmwOpenCfg;

    /*! @brief  Chirping center frequency */
    float                               centerFreq;

    /*! @brief  Rx Channel offsets in ADC buffer - not used */
    uint16_t                            rxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

    /*! @brief  Range processing DPU handle */
    DPU_RangeProcHWA_Handle             rangeProcDpuHandle;

    /*! @brief  Doppler processing DPU handle */
    DPU_DopplerProcHWA_Handle           dopplerProcDpuHandle;

    /*! @brief  CFAR DPU handle */
    DPU_CFARProcHWA_Handle              cfarProcDpuHandle;

    /*! @brief  AOA2D DPU handle */
    DPU_AoAProcHWA_Handle                aoa2dProcDpuHandle;

    /* DPC configs from CLI */
    /*! @brief Number of enabled Tx antennas */
    uint16_t                            numTxAntennas;
    uint8_t                             txAntOrder[SYS_COMMON_NUM_TX_ANTENNAS];

    /*! @brief Number of enabled Rx antennas */
    uint16_t                            numRxAntennas;
    
    uint8_t                             rxAntOrder[SYS_COMMON_NUM_RX_CHANNEL];
    
    float                               adcStartTime;

    /*! @brief ADC sampling rate in MHz */
    float                               adcSamplingRate;

    /*! @brief Number of range bins */
    uint32_t                            numRangeBins;

    /*! @brief Number of Doppler bins */
    uint32_t                            numDopplerBins;

    MmwDemo_antennaGeometryCfg          antennaGeometryCfg;
    MmwDemo_antennaGeometryCfg          activeAntennaGeometryCfg;
    uint16_t                            numAntRow;
    uint16_t                            numAntCol;

    /*! @brief Flag: 0-Low power mode disabled, 1-Low Power mode enabled, 2-Used for testing low power mode */
    uint8_t                             lowPowerMode;

    /*! @brief Flag to control in low power mode some configuration parts to be executed only once, and not to be repeated from frame to frame */
    uint8_t                             oneTimeConfigDone;

    /*! @brief L3 ram memory pool object */
    MemPoolObj    L3RamObj;

    /*! @brief Core Local ram memory pool object */
    MemPoolObj    CoreLocalRamObj;

    /*! @brief Memory Usage */
    DPC_memUsage    memUsage;

    /*! @brief ADC buffer allocated in testing mode only */
    uint8_t         *adcTestBuff;

    /*! @brief      Radar cube data interface for [0]-Major Motion Detection [1]-Minor Motion Detection*/
    cmplx16ImRe_t   *radarCube;

    /*! @brief      Detection matrix */
    uint16_t        *detMatrix;

    /*! @brief      Range profiles to be exported to GUI */
    uint32_t        *rangeProfile;

    /*! @brief      Doppler index matrix, type uint8, size = number range bins x number azimuth bins * number of elevation bins */
    DPIF_DetMatrix dopplerIndexMatrix;

    /*! @brief      Elevation index matrix, type uint8, size = number range bins x number azimuth bins*/
    DPIF_DetMatrix elevationIndexMatrix;

    /*! @brief      Pointers to DPC output data */
    DPC_ObjectDetection_ExecuteResult dpcResult;

    /*! @brief Point cloud structure with int16 type coordinates sent to Host via UART. Includes TLV header and uints structure */
    MmwDemo_output_message_UARTpointCloud pointCloudToUart;

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

        /*! @brief  Output Point cloud list from AoA DPU */
    DPIF_PointCloudCartesian *dpcAoAObjOut;

    /*! @brief  Output Point cloud side information list from AoA DPU */
    DPIF_PointCloudSideInfo *dpcAoAObjSideInfo;

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

    /*! @brief HWA DMA trigger source channel pool */
    HwaDmaTrigChanPoolObj  HwaDmaChanPoolObj;

    /*! @brief HWA Window RAM memory pool */
    HwaWinRamMemoryPoolObj  HwaWinRamMemoryPoolObj;

    /*! @brief Number of used HWA param sets */
    uint8_t numUsedHwaParamSets;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t        factoryCalibFlashOffset;

    /*! @brief   LVDS streaming configuration */
    MmwDemo_LVDSStream_MCB_t    lvdsStream;

    /*! @brief   ADC data dithering enable flag */
    bool adcDataDithEnable;

} MmwDemo_MSS_MCB;

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, on the XWRL684x platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum MmwDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    /*! @brief   Samples to calculate static azimuth/elevation heatmap, (all virtual antennas exported) */
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,

    /*! @brief   temperature stats from Radar front end */
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,

    MMWDEMO_OUTPUT_MSG_MAX,

    MMWDEMO_OUTPUT_EXT_MSG_START = 300,

    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS,

    /*! @brief   Range profile - Major Motion */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR,

    /*! @brief   Range-azimuth  heatmap - Major motion */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MAJOR,

    /*! @brief   Range-azimuth  heatmap - Minor motion  */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MINOR,

    /*! @brief   Stats information, (timing, temperature, power) */
    MMWDEMO_OUTPUT_EXT_MSG_STATS,

    /*! @brief   Presence information */
    MMWDEMO_OUTPUT_EXT_MSG_PRESENCE_INFO,

    /*! @brief   Target List - Array of detected targets (position, velocity, error covariance) */
    MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST,

    /*! @brief   Target List - Array of target indices */
    MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX,

    /*! @brief   Micro doppler raw data */
    MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA,

    /*! @brief   Micro doppler features */
    MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES,

    /*! @brief   Radar Cube - Major motion */
    MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MAJOR,

    /*! @brief   Radar Cube - Minor motion  */
    MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MINOR,

    /*! @brief   Point cloud range azimuth indices */
    MMWDEMO_OUTPUT_EXT_MSG_POINT_CLOUD_INDICES,

    /*! @brief   Presence detected in a region of interest - major, minor, unoccupied  */
    MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION,

    /*! @brief   ADC samples */
    MMWDEMO_OUTPUT_EXT_MSG_ADC_SAMPLES,

    /*! @brief   Classifier info */
    MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO,

    /*! @brief   Rx Channel compensation info */
    MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO,

    MMWDEMO_OUTPUT_EXT_MSG_MAX

} MmwDemo_output_message_type;

#define MMWDEMO_OUTPUT_ALL_MSG_MAX (MMWDEMO_OUTPUT_MSG_MAX + MMWDEMO_OUTPUT_EXT_MSG_MAX - MMWDEMO_OUTPUT_EXT_MSG_START - 1)

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

    /*! @brief   Time in CPU cycles when the message was created.*/
    uint32_t    timeCpuCycles;

    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObj;

    /*! @brief   Number of TLVs */
    uint32_t    numTLVs;

    uint32_t    subFrameNumber;

} MmwDemo_output_message_header;

/* Debug Functions */
extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_debugAssert(expression) {                                      \
                                         _MmwDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }

extern void mmWDemo_parkLvdsPins();
#ifdef __cplusplus
}
#endif

#endif /* MMWAVE_DEMO_H */