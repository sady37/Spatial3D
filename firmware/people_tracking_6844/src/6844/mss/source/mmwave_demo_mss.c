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
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* mmwave SDK files */
#include <datapath/dpedma/v1/dpedmahwa.h>
#include <datapath/dpedma/v1/dpedma.h>
#include <datapath/dpu/rangeproc/v1/rangeprochwa.h>
#include <datapath/dpu/rangeproc/v1/rangeprochwa_internal.h>
#include <datapath/dpu/dopplerproc/v1/dopplerprochwa.h>
#include <datapath/dpu/aoa2dproc/v1/aoa2dproc.h>
#include <utils/mathutils/mathutils.h>

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/edma.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/cbuff.h>
#include <board/ina.h>
#include <drivers/mcspi.h>
#include <drivers/power.h>
#include <drivers/prcm.h>

#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>

/* PMIC LLD include */
#include <board/pmic/include/pmic.h>

#include <source/common_test.h>
#include <source/mmw_res.h>
#include <source/mmw_cli.h>
#include <source/mmwave_demo_mss.h>
#include <source/calibrations/mmw_flash_cal.h>
#include <source/dpc/dpc_mss.h>
#include <source/calibrations/range_phase_bias_measurement.h>
#include <source/utils/mmw_demo_utils.h>
#include <source/power_management/power_management.h>
#include <source/lvds_streaming/mmw_lvds_stream.h>
#include <source/calibrations/factory_cal.h>
#include <source/mmwave_control/monitors.h>


#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"

/**************************************************************************
 *************************** Macros Definitions ***************************
 **************************************************************************/
#define MMW_DEMO_SYNC_POINT_CLOUD_AND_PREDICTIONS


/* Enable Continuous wave mode */
#define CONTINUOUS_WAVE_MODE_ENABLE   0

/* Frame ref timer clock for stats info */
#define FRAME_REF_TIMER_CLOCK_MHZ  40

// Time to transfer single Byte of data obtained by measuring time for various data size.
#define TIME_TO_SEND_1BYTE_DATA_WITH_BAUDRATE_115200_us  95  
#define TIME_TO_SEND_1BYTE_DATA_WITH_BAUDRATE_1250000_us  10
#define TIME_TO_SEND_1BYTE_DATA_WITH_BAUDRATE_921600_us  14

// Task specific defines
#define POWER_TASK_PRI  (2u)
#define POWER_TASK_SIZE (1024u)
#define MMWINITTASK_PRI  (5u)
#define MMWINIT_TASK_SIZE (1024u)

// Low power mode defines
#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)


/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
/*! Low Power Mode Latency Start time */
unsigned long long              ll_LPmode_LatencyStart = 0;

/*! Low Power Mode Latency End time */
unsigned long long              ll_LPmode_LatencyEnd = 0;

/*! MSS Demo Master Configurations Structure */
MmwDemo_MSS_MCB                 gMmwMssMCB = {0};

/*! Default antenna geometry - xwrL6844 EVM */
MmwDemo_antennaGeometryCfg      gDefaultAntGeometry = {.ant = {{3,2}, {2,2}, {2,3}, {3,3}, {3,0}, {2,0}, {2,1}, {3,1}, {1,0}, {0,0}, {0,1}, {1,1}, {1,2}, {0,2}, {0,3}, {1,3}}};

/*! L3 RAM buffer for object detection DPC */
uint8_t                         gMmwL3[MSS_L3_MEM_SIZE]  __attribute((section(".bss.l3")));

/*! Local RAM buffer for object detection DPC */
uint8_t                         gMmwCoreLocMem[MSS_CORE_LOCAL_MEM_SIZE];

/*! Local2 RAM buffer size */
uint8_t gMmwCoreLocMem2[MSS_CORE_LOCAL_MEM2_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/*! HWA driver instance handle */
HWA_Handle                      gHwaHandle;

/*! Temperature stats info */
MMWave_temperatureStats         gTempStats;

/*! Task specific declarations */
StaticTask_t                    gMmwInitTaskObj;
TaskHandle_t                    gMmwInitTask;
StackType_t                     gMmwInitTaskStack[MMWINIT_TASK_SIZE] __attribute__((aligned(32)));
StaticSemaphore_t               gMmwInitObj;
SemaphoreHandle_t               gMmwInit;

StackType_t                     gPowerTaskStack[POWER_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t                    gPowerTaskObj;
TaskHandle_t                    gPowerTask;
StaticSemaphore_t               gPowerSemObj;
SemaphoreHandle_t               gPowerSem;

TaskHandle_t                    gDpcTask;
StaticTask_t                    gDpcTaskObj;
StackType_t                     gDpcTaskStack[DPC_TASK_STACK_SIZE] __attribute__((aligned(32)));

TaskHandle_t                    gTlvTask;
StaticTask_t                    gTlvTaskObj;
StackType_t                     gTlvTaskStack[TLV_TASK_STACK_SIZE] __attribute__((aligned(32)));

TaskHandle_t                    gAdcFileTask;
StaticTask_t                    gAdcFileTaskObj;
StackType_t                     gAdcFileTaskStack[ADC_FILEREAD_TASK_STACK_SIZE] __attribute__((aligned(32)));

TaskHandle_t                    gDspPointCloudTask;
StaticTask_t                    gDspPointCloudTaskObj;
/* Spatial3D FIX2: classifier task stack pinned to TCMB (.bss.dsp_tcmb, see linker.cmd)
 * to keep TCMA off the razor edge - see the fallsm-boot-bug note. Kept even though the
 * fall SM was removed in Phase 1 (TCMB has ample room; this is pure headroom insurance). */
StackType_t                     gDspPointCloudTaskStack[CLASSIFIER_TASK_STACK_SIZE] __attribute__((aligned(32), section(".bss.dsp_tcmb")));

/*! LED configurations */
uint32_t                        gGpioBaseAddrLed, gPinNumLed;

/* For Sensor Stop */
uint32_t                        gSensorStop = 0;

double                          gDemoTimeus, gFrmPrdus;

volatile unsigned long long     gSlpTimeus, gLpdsLatency;

float                           gSocClk = 40000000; //Hz


Pmic_CoreHandle_t pmicHandle;

/* For freeing the channels after Sensor Stop */
void MmwDemo_freeDmaChannels(EDMA_Handle edmaHandle);

void MmwDemo_transmitProcessedOutputTask();

void MmwDemo_uartWrite (UART_Handle handle,
                            uint8_t *payload,
                            uint32_t payloadLength);

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern uint8_t                       gIsSensorStarted;
extern TaskHandle_t                 gDpcTask;
extern TaskHandle_t                 gAdcFileTask;
// extern DPU_MacroDopplerProc_Config  gMacroDoppProcDpuCfg;

//extern void MmwDemo_reinitTask(void *args);
//extern void MmwDemo_getTemperatureReport (MMWave_temperatureStats* ptrTempStats);
extern void MmwDemo_populateControlCfg ();
extern void MmwDemo_dpcTask();
extern void MmwDemo_adcFileReadTask();
extern void Power_enablePolicy();

#if (ENABLE_MONITORS==1)
/*API to get Results of RF Monitors*/
extern void MmwDemo_getMonRes(void);
#endif

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\r\n",file,line);
    }
}

// Free all the allocated EDMA channels
void mmwDemo_freeDmaChannels(EDMA_Handle edmaHandle)
{
    uint32_t   index;
    uint32_t  dmaCh, tcc, pram, shadow;
    for(index = 0; index < 64; index++)
    {
        dmaCh = index;
        tcc = index;
        pram = index;
        shadow = index;
        DPEDMA_freeEDMAChannel(edmaHandle, &dmaCh, &tcc, &pram, &shadow);
    }
    for(index = 0; index < 128; index++)
    {
        shadow = index;
        DebugP_assert(EDMA_freeParam(edmaHandle, &shadow) == SystemP_SUCCESS);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      MMW Demo helper Function to Stop the Sensor. Sensor Stop in honored only when Low Power Mode is disabled.
 *
 *  @retval
 *      None
 */
void MmwDemo_stopSensor(void)
{
    int32_t err;
    // Stop and Close the front end
    MMWave_stop(gMmwMssMCB.ctrlHandle, &err); /* spatial3d: SDK6.0.4.1 MMWave_stop 2-arg */
    //MMWave_close(gMmwMssMCB.ctrlHandle,&err);
    if (gMmwMssMCB.adcLogging.enable == 1)
    {
        if(gMmwMssMCB.lvdsStream.sessionHandle != NULL)
        {
            MmwDemo_LVDSStreamDeleteSession();
        }
    }
    // Free up all the edma channels and close the EDMA interface
    mmwDemo_freeDmaChannels(gEdmaHandle[0]);
    Drivers_edmaClose();
    EDMA_deinit();
    // Demo Stopped
    rangeProcHWAObj* temp = gMmwMssMCB.rangeProcDpuHandle;
    temp->inProgress = false;
    gMmwMssMCB.oneTimeConfigDone = 0;
    // Re-init the EDMA interface for next configuration
    EDMA_init();
    Drivers_edmaOpen();
    gMmwMssMCB.stats.frameStartIntCounter = 0;
    gSensorStop = 0;
    gIsSensorStarted = 0;
    gMmwMssMCB.oneTimeConfigDone = 0;
    gMmwMssMCB.mmWaveCfg.initCfg.iswarmstart = FALSE;
    // gMmwMssMCB.sbrCliCurrentCuboidInd = 0;
    // gMmwMssMCB.sbrCliCurrentZoneInd = 0;
    // memset(&gMmwMssMCB.featureExtrModuleCfg.sceneryParams, 0, sizeof(FEXTRACT_sceneryParams));

    // Delete the DPC, TLV as we will create them again in next configuration when we start
    vTaskDelete(gDpcTask);
    if (gDspPointCloudTask)
    {
        vTaskDelete(gDspPointCloudTask);
        gDspPointCloudTask = NULL;
    }
    vTaskDelete(NULL);
}

/**************************************************************************
 *************************** Static Definitions ***************************
 **************************************************************************/
/* Helper function for MMWave_init() control lib */
int32_t MmwDemo_mmWaveInit(bool iswarmstrt)
{
    int32_t             errCode;
    int32_t             retVal = SystemP_SUCCESS;
    ADCBuf_Params       adcBuffParams;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;

    /* Initialize the mmWave control init configuration */
    memset ((void*)&gMmwMssMCB.mmWaveCfg.initCfg, 0, sizeof(MMWave_InitCfg));

    /* Is Warm Start? */
    gMmwMssMCB.mmWaveCfg.initCfg.iswarmstart = iswarmstrt;

    /* Open the first ADCBUF Instance */
    ADCBuf_Params_init(&adcBuffParams);
    gMmwMssMCB.adcBuffHandle = ADCBuf_open(CONFIG_ADCBUF0, &adcBuffParams);

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&gMmwMssMCB.mmWaveCfg.initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Error: Unable to initialize the mmWave control module */
        CLI_write ("Error: mmWave Control Initialization failed [Error code %d] [errorLevel %d] [mmWaveErrorCode %d] [subsysErrorCode %d]\n", errCode, errorLevel, mmWaveErrorCode, subsysErrorCode);
        retVal = SystemP_FAILURE;
    }
    /* FECSS RF Power ON*/
    if(gMmwMssMCB.mmWaveCfg.initCfg.iswarmstart)
    {
        /* FECSS RF Power ON*/
        retVal = MMWave_FecssRfPwrOnOff(gMmwMssMCB.mmWaveCfg.txEnbl, gMmwMssMCB.mmWaveCfg.rxEnbl, &errCode);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            CLI_write ("Error: FECSS RF Power ON/OFF failed during warm reset\r\n");
            retVal = SystemP_FAILURE;
            MmwDemo_debugAssert (0);
        }   
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *     UART write wrapper function
 *
 * @param[in]   handle          UART handle
 * @param[in]   payload         Pointer to payload data
 * @param[in]   payloadLength   Payload length in bytes
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_uartWrite (UART_Handle handle,
                            uint8_t *payload,
                            uint32_t payloadLength)
{
    UART_Transaction trans;

    UART_Transaction_init(&trans);

    trans.buf   = payload;
    trans.count = payloadLength;

    UART_write(handle, &trans);
}

void MmwDemo_inaMeasNull(I2C_Handle i2cHandle, uint16_t *ptrPwrMeasured)
{
    ptrPwrMeasured[0] = (uint16_t)0xFFFF;
    ptrPwrMeasured[1] = (uint16_t)0xFFFF;
    ptrPwrMeasured[2] = (uint16_t)0xFFFF;
    ptrPwrMeasured[3] = (uint16_t)0xFFFF;
}

/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and including the number of TLV items
*    TLV Items:
*    2. If pointCloud flag is 1 or 2, DPIF_PointCloudCartesian structure containing
*       X,Y,Z location and velocity for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    3. If pointCloud flag is 1, DPIF_PointCloudSideInfo structure containing SNR
*       and noise for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    4. If rangeProfile flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint32_t)
*    5. noiseProfile flag is set is not used.
*    6. If rangeAzimuthHeatMap flag is set, sends range/azimuth heatmap, size = number of range bins *
*       number of azimuth bins * sizeof(uint32_t)
*    7. rangeDopplerHeatMap flag is not used
*    8. If statsInfo flag is set, the stats information, timing, temperature and power
*/
volatile uint32_t gDbgRangeOffset = 22;
void MmwDemo_transmitProcessedOutputTask()
{
    UART_Handle uartHandle = gUartHandle[1];
    //MmwDemo_output_message_stats      *timingInfo
    MmwDemo_output_message_headerID headerID;
    MmwDemo_GuiMonSel   *pGuiMonSel;
    uint32_t tlvIdx = 0;

    MmwDemo_output_message_UARTpointCloud *objOut= &gMmwMssMCB.pointCloudToUart;
    DPC_ObjectDetection_ExecuteResult *majorResult = &gMmwMssMCB.dpcResult;
    uint8_t                            trackerEnabled;
    uint32_t                           numTargets, numIndices;
    uint8_t                           *tList;
    uint8_t                           *tIndex;

    I2C_Handle  i2cHandle = gI2cHandle[CONFIG_I2C0];

    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_ALL_MSG_MAX];

    /* Save/restore FP registers during the context switching */
    vPortTaskUsesFPU();

    /* Get Gui Monitor configuration */
    pGuiMonSel = &gMmwMssMCB.guiMonSel;
    trackerEnabled = gMmwMssMCB.trackerCfg.staticCfg.trackerEnabled;

    /* Send signal to CLI task that this is ready */
    SemaphoreP_post(&gMmwMssMCB.uartTaskConfigDoneSemHandle);

    while(true)
    {
        SemaphoreP_pend(&gMmwMssMCB.tlvSemHandle, SystemP_WAIT_FOREVER);

        /* Begin of UART data transmission */
        DPC_ObjectDetection_Profile(&gMmwMssMCB.stats.uartTransStart);

        tlvIdx = 0;
        objOut      = &(gMmwMssMCB.pointCloudToUart);

        /* Clear message header */
        memset((void *)&headerID, 0, sizeof(MmwDemo_output_message_headerID));
        /* Header: */
        headerID.platform =  0xA6844;
        headerID.magicWord[0] = 0x0102;
        headerID.magicWord[1] = 0x0304;
        headerID.magicWord[2] = 0x0506;
        headerID.magicWord[3] = 0x0708;
        headerID.numDetectedObjMajor = majorResult->numObjOut;
        headerID.numDetectedObjMinor = gMmwMssMCB.numDetectedPointsMinor;
        headerID.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                            (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                            (MMWAVE_SDK_VERSION_MINOR << 16) |
                            (MMWAVE_SDK_VERSION_MAJOR << 24);

        /* Tracker information */
        numTargets = majorResult->trackerOutParams.numTargets;
        numIndices = majorResult->trackerOutParams.numIndices;
        tList      = (uint8_t *)majorResult->trackerOutParams.tList;
        tIndex     = (uint8_t *)majorResult->trackerOutParams.targetIndex;

        packetLen = sizeof(MmwDemo_output_message_header);

        /*** Point cloud ***/
        if ((pGuiMonSel->pointCloud) && (headerID.numDetectedObjMinor > 0))
        {

            // Minor
            packetLen += sizeof(MmwDemo_output_message_tl) + objOut->messageTL.length ;
            tlvIdx++;

        }

        if ((pGuiMonSel->pointCloud) && (headerID.numDetectedObjMajor > 0))
        {

            // Major
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
            tl[tlvIdx].length = sizeof(DPIF_PointCloudCartesian) * majorResult->numObjOut;
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /*********************************/
        /* Range Profile */
        /*********************************/
        if ((pGuiMonSel->rangeProfile & 0x1) && (gMmwMssMCB.rangeProfile != NULL))
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
            tl[tlvIdx].length = sizeof(uint32_t) * gMmwMssMCB.numRangeBins;
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /****************************************/
        /* Rx channel compensation coefficients */
        /****************************************/
        if (gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO;
            tl[tlvIdx].length = sizeof(DPC_ObjDet_compRxChannelBiasFloatCfg);
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /****************************************/
        /* Tracker Results                      */
        /****************************************/

        if (trackerEnabled && pGuiMonSel->trackerInfo)
        {
            if (numTargets > 0)
            {
                tl[tlvIdx].type   = MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST;
                tl[tlvIdx].length = numTargets * sizeof(trackerProc_Target);
                packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
                tlvIdx++;
            }
            if ((numIndices > 0) && (numTargets > 0))
            {
                tl[tlvIdx].type   = MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX;
                tl[tlvIdx].length = numIndices * sizeof(trackerProc_TargetIndex);
                packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
                tlvIdx++;
            }
        }

        /****************************************/
        /* Spatial3D: range-window bin cube     */
        /* (armed by cubeQuery; tbcNumEntries>0  */
        /*  only while a query burst is active)  */
        /****************************************/
        if (gMmwMssMCB.tbcNumEntries > 0)
        {
            tl[tlvIdx].type   = MMWDEMO_OUTPUT_EXT_MSG_TRACK_BIN_CUBE;
            tl[tlvIdx].length = 2 * sizeof(uint16_t) +
                (uint32_t)gMmwMssMCB.tbcNumEntries *
                (sizeof(uint32_t) + 2 * sizeof(uint16_t) + sizeof(float) +
                 (uint32_t)gMmwMssMCB.tbcNumVirtAnt * sizeof(cmplx16ImRe_t));
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /****************************************/
        /* Spatial3D: per-track pose (TLV 321)  */
        /****************************************/
        if (gMmwMssMCB.poseEnable && gMmwMssMCB.poseNumResults > 0)
        {
            tl[tlvIdx].type   = MMWDEMO_OUTPUT_EXT_MSG_POSE;
            tl[tlvIdx].length = 2 * sizeof(uint16_t) +
                (uint32_t)gMmwMssMCB.poseNumResults * sizeof(PoseResult);
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /****************************************/
        /* Stats                                */
        /****************************************/
        if (gMmwMssMCB.guiMonSel.statsInfo)
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
            tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /* Fill header */
        headerID.numTLVs = tlvIdx;
        /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
        headerID.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
                ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
        headerID.timeCpuCycles =  0; //TODO: Populate with actual time
        headerID.frameNumber = gMmwMssMCB.stats.frameStartIntCounter;
        headerID.subFrameNumber = -1;

        /****************/
        /* Write header */
        /****************/
        MmwDemo_uartWrite (uartHandle, (uint8_t*)&headerID, sizeof(MmwDemo_output_message_headerID));
        tlvIdx = 0;


        /***************************************************/
        /* Send Point Cloud, feature extraction Classifier */
        /***************************************************/
        if ((pGuiMonSel->pointCloud) && (headerID.numDetectedObjMinor  > 0))
        {
            // Minor
            /* Send point cloud */
            MmwDemo_uartWrite (uartHandle, (uint8_t*)&gMmwMssMCB.pointCloudToUart,
                        sizeof(MmwDemo_output_message_tl) + objOut->messageTL.length);
            tlvIdx++;
        }

        if ((pGuiMonSel->pointCloud) && (headerID.numDetectedObjMajor  > 0))
        {
            // Major
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&tl[tlvIdx],
                            sizeof(MmwDemo_output_message_tl));

            /*Send array of objects */
            MmwDemo_uartWrite (uartHandle, (uint8_t*)majorResult->objOut,
                            sizeof(DPIF_PointCloudCartesian) * majorResult->numObjOut);
            tlvIdx++;
        }

        /****************************************/
        /* Send Range profile                   */
        /****************************************/
        if ((pGuiMonSel->rangeProfile & 0x1) && (gMmwMssMCB.rangeProfile != NULL))
        {
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&tl[tlvIdx],
                            sizeof(MmwDemo_output_message_tl));

            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)gMmwMssMCB.rangeProfile,
                            sizeof(uint32_t) * gMmwMssMCB.numRangeBins);
            tlvIdx++;
        }

        /* Send Group Tracker output */
        if (trackerEnabled && (pGuiMonSel->trackerInfo == 1))
        {
            if (numTargets > 0)
            {
                MmwDemo_uartWrite(uartHandle,
                              (uint8_t *)&tl[tlvIdx],
                              sizeof(MmwDemo_output_message_tl));
                MmwDemo_uartWrite(uartHandle,
                              (uint8_t *)tList,
                              tl[tlvIdx].length);
                tlvIdx++;
            }
            if ((numIndices > 0) && (numTargets > 0))
            {
                MmwDemo_uartWrite(uartHandle,
                              (uint8_t *)&tl[tlvIdx],
                              sizeof(MmwDemo_output_message_tl));
                MmwDemo_uartWrite(uartHandle,
                              (uint8_t *)tIndex,
                              tl[tlvIdx].length);
                tlvIdx++;
            }
        }

        /*********************************************/
        /* Spatial3D: Send range-window per-bin cube */
        /*********************************************/
        if (gMmwMssMCB.tbcNumEntries > 0)
        {
            uint16_t subHdr[2];
            uint16_t k;
            MmwDemo_uartWrite(uartHandle, (uint8_t *)&tl[tlvIdx],
                              sizeof(MmwDemo_output_message_tl));
            subHdr[0] = gMmwMssMCB.tbcNumEntries;
            subHdr[1] = gMmwMssMCB.tbcNumVirtAnt;
            MmwDemo_uartWrite(uartHandle, (uint8_t *)subHdr, sizeof(subHdr));
            for (k = 0; k < gMmwMssMCB.tbcNumEntries; k++)
            {
                MmwDemo_TrackBinEntry *ent = &gMmwMssMCB.tbcEntries[k];
                MmwDemo_uartWrite(uartHandle, (uint8_t *)&ent->tid, sizeof(uint32_t));
                MmwDemo_uartWrite(uartHandle, (uint8_t *)&ent->rangeBin, sizeof(uint16_t));
                MmwDemo_uartWrite(uartHandle, (uint8_t *)&ent->velMag_mmps, sizeof(int16_t));
                MmwDemo_uartWrite(uartHandle, (uint8_t *)&ent->range_m, sizeof(float));
                MmwDemo_uartWrite(uartHandle, (uint8_t *)ent->vec,
                                  (uint32_t)gMmwMssMCB.tbcNumVirtAnt * sizeof(cmplx16ImRe_t));
            }
            tlvIdx++;
        }

        /*********************************************/
        /* Spatial3D: Send per-track pose (TLV 321)  */
        /*********************************************/
        if (gMmwMssMCB.poseEnable && gMmwMssMCB.poseNumResults > 0)
        {
            uint16_t poseHdr[2];
            MmwDemo_uartWrite(uartHandle, (uint8_t *)&tl[tlvIdx],
                              sizeof(MmwDemo_output_message_tl));
            poseHdr[0] = gMmwMssMCB.poseNumResults;
            poseHdr[1] = 0;   /* reserved (keeps the entry array 4-byte aligned) */
            MmwDemo_uartWrite(uartHandle, (uint8_t *)poseHdr, sizeof(poseHdr));
            MmwDemo_uartWrite(uartHandle, (uint8_t *)gMmwMssMCB.poseResults,
                              (uint32_t)gMmwMssMCB.poseNumResults * sizeof(PoseResult));
            tlvIdx++;
        }

        /*********************************************/
        /* Send Rx Channel compensation coefficients */
        /*********************************************/
        if (gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
        {
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&tl[tlvIdx],
                            sizeof(MmwDemo_output_message_tl));
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&gMmwMssMCB.compRxChannelBiasCfgMeasureOut,
                            tl[tlvIdx].length);
            tlvIdx++;
        }

        /****************************************/
        /* Stats                                */
        /****************************************/
        if (gMmwMssMCB.guiMonSel.statsInfo)
        {
#if 1
            gMmwMssMCB.outStats.tempReading[0] = 0;
            gMmwMssMCB.outStats.tempReading[1] = 0;
            for(int i=0; i<4; i++)
            {
                gMmwMssMCB.outStats.tempReading[0]+=gTempStats.tempValue[i];
                gMmwMssMCB.outStats.tempReading[1]+=gTempStats.tempValue[i+4];
            }
            gMmwMssMCB.outStats.tempReading[0] = gMmwMssMCB.outStats.tempReading[0]/4; // Average of all Rx temp
            gMmwMssMCB.outStats.tempReading[1] = gMmwMssMCB.outStats.tempReading[1]/4; // Average of all Tx temp
            gMmwMssMCB.outStats.tempReading[2] = gTempStats.tempValue[8]; // PM temp
            gMmwMssMCB.outStats.tempReading[3] = gTempStats.tempValue[9]; // DIG temp
#endif
            mmwDemo_PowerMeasurement(i2cHandle, &gMmwMssMCB.outStats.powerMeasured[0]);

            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&tl[tlvIdx],
                            sizeof(MmwDemo_output_message_tl));
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&gMmwMssMCB.outStats,
                            tl[tlvIdx].length);
            tlvIdx++;
        }

        /*** Capon heatmap ***/
        if (gMmwMssMCB.dbgGuiMonSel.exportCoarseHeatmap)
        {
            //Header
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&tl[tlvIdx],
                            sizeof(MmwDemo_output_message_tl));

            //numRangeBins, numAzimuthBins, numElevBins
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&gMmwMssMCB.outputFromDSP->heatMapOut,
                            3*sizeof(uint32_t));

            //heatmap
            {
                uint8_t *data = (uint8_t*)gMmwMssMCB.outputFromDSP->heatMapOut.data;
                int32_t startInd, sendLen;
                int32_t chunkLen = 800;
                int32_t len =  gMmwMssMCB.outputFromDSP->heatMapOut.numRangeBins *
                                gMmwMssMCB.outputFromDSP->heatMapOut.numAzimuthBins *
                                gMmwMssMCB.outputFromDSP->heatMapOut.numElevationBins * sizeof(float);

                for (startInd = 0; startInd < len; startInd += chunkLen)
                {
                    if ((len - startInd) > chunkLen)
                    {
                        sendLen = chunkLen;
                    }
                    else
                    {
                        sendLen = len - startInd;
                    }
                    MmwDemo_uartWrite (uartHandle, &data[startInd], sendLen);
                }
            }
            tlvIdx++;
        }
        /*** coarse point  cloud ***/
        if (gMmwMssMCB.dbgGuiMonSel.exportRawCfarDetList)
        {
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&tl[tlvIdx],
                            sizeof(MmwDemo_output_message_tl));

            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&gMmwMssMCB.outputFromDSP->rawCfarPointCloud.object_count,
                            sizeof(uint32_t));
            MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)gMmwMssMCB.outputFromDSP->rawCfarPointCloud.list,
                            sizeof(DPIF_MSS_DSS_rawCfarDetPoint)*gMmwMssMCB.outputFromDSP->rawCfarPointCloud.object_count);
            tlvIdx++;
        }

        if(tlvIdx != 0)
        {
            /* Send padding bytes */
            numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
            if (numPaddingBytes < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
            {
                MmwDemo_uartWrite (uartHandle, (uint8_t*)padding, numPaddingBytes);
            }
        }
        /* Flush UART buffer here for each frame. */
        UART_flushTxFifo(uartHandle);

        gMmwMssMCB.frmCntrInSlidingWindowUart++;
        if (gMmwMssMCB.frmCntrInSlidingWindowUart == gMmwMssMCB.sigProcChainCommonCfg.numFrmPerSlidingWindow)
        {
            gMmwMssMCB.frmCntrInSlidingWindowUart = 0;
        }

        /* End of UART data transmission */
        DPC_ObjectDetection_Profile(&gMmwMssMCB.stats.uartTransCompletion);

        gMmwMssMCB.outStats.transmitOutputTime = gMmwMssMCB.stats.uartTransCompletion.timeInUsec - gMmwMssMCB.stats.uartTransStart.timeInUsec;

        //Interframe processing and UART data transmission completed
        gMmwMssMCB.interSubFrameProcToken--;

        #if (CLI_REMOVAL == 0)
        if (gMmwMssMCB.adcDataSourceCfg.source == 1 || gMmwMssMCB.adcDataSourceCfg.source == 2)
        {
            gDemoTimeus = 0;
        }
        #endif

        if (gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_DISABLE)
        {
            #if (CLI_REMOVAL == 0)
            if(gMmwMssMCB.adcDataSourceCfg.source == 1 || gMmwMssMCB.adcDataSourceCfg.source == 2)
            {
                /* In test mode trigger next frame processing */
                SemaphoreP_post(&gMmwMssMCB.adcFileTaskSemHandle);
            }
            #endif
            // Important Note: Sensor Stop command is honored only when Low Power Cfg is disabled
            if(gSensorStop == 1)
            {
                MmwDemo_stopSensor();
            }
        }

        if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE) || (gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE))
        {
            xSemaphoreGive(gPowerSem);
            /* Enable Power Management Policy if Low Power Mode is enabled */
            if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
            {
                Power_enablePolicy();
            }
        }


    }
}

volatile float oneQ7float = 128.0;
#if 0
#endif
volatile float oneOver8 = 0.125;

void MmwDemo_dspPointCloudTask()
{
    int32_t pntIdx;
    DPIF_MSS_DSS_radarProcessOutput *outputFromDSP;

    /* Save/restore FP registers during the context switching */
    vPortTaskUsesFPU();

    gMmwMssMCB.pointCloudToUart.messageTL.type             = MMWDEMO_OUTPUT_MSG_POINT_CLOUD;
    gMmwMssMCB.pointCloudToUart.pointUnit.azimuthUnit   = (PI/2.f)/127.f;
    gMmwMssMCB.pointCloudToUart.pointUnit.elevationUnit = (PI/2.f)/127.f;
    gMmwMssMCB.pointCloudToUart.pointUnit.rangeUnit     = 0.00025f;
    gMmwMssMCB.pointCloudToUart.pointUnit.dopplerUnit   = 0.00028f;
    gMmwMssMCB.pointCloudToUart.pointUnit.snrUint       = 1./256.;//snr from DSP is in dB in Q8 format

    gMmwMssMCB.pointUnitInv.azimuthUnit   = 1./gMmwMssMCB.pointCloudToUart.pointUnit.azimuthUnit;
    gMmwMssMCB.pointUnitInv.elevationUnit = 1./gMmwMssMCB.pointCloudToUart.pointUnit.elevationUnit;
    gMmwMssMCB.pointUnitInv.rangeUnit     = 1./gMmwMssMCB.pointCloudToUart.pointUnit.rangeUnit;
    gMmwMssMCB.pointUnitInv.dopplerUnit   = 1./gMmwMssMCB.pointCloudToUart.pointUnit.dopplerUnit;
    gMmwMssMCB.pointUnitInv.snrUint      = 1./gMmwMssMCB.pointCloudToUart.pointUnit.snrUint;
    /* Wait for point cloud */
    SemaphoreP_pend(&gMmwMssMCB.classifierTaskSemHandle, SystemP_WAIT_FOREVER);

    DPC_ObjectDetection_Profile(&gMmwMssMCB.stats.pointCloudCompletion);

    // copy to the format for output, and to future tracker
    outputFromDSP = gMmwMssMCB.outputFromDSP;
    // outClassifierProc = &gMmwMssMCB.classifierResult;
    gMmwMssMCB.numDetectedPointsMinor = outputFromDSP->pointCloudOut.object_count;
    gMmwMssMCB.pointCloudToUart.messageTL.length = sizeof(MmwDemo_output_message_point_unit) + sizeof(MmwDemo_output_message_UARTpoint) * outputFromDSP->pointCloudOut.object_count;
    if (gMmwMssMCB.numDetectedPointsMinor == 0)
        gMmwMssMCB.pointCloudToUart.messageTL.length = 0;
    for (pntIdx = 0; pntIdx < (int32_t)outputFromDSP->pointCloudOut.object_count; pntIdx++)
    {
        gMmwMssMCB.pointCloudToUart.point[pntIdx].azimuth   = (int8_t)round(outputFromDSP->pointCloudOut.pointCloud[pntIdx].azimuthAngle * gMmwMssMCB.pointUnitInv.azimuthUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].elevation = (int8_t)round((outputFromDSP->pointCloudOut.pointCloud[pntIdx].elevAngle) * gMmwMssMCB.pointUnitInv.elevationUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].range     = (uint16_t)round(outputFromDSP->pointCloudOut.pointCloud[pntIdx].range * gMmwMssMCB.pointUnitInv.rangeUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].doppler   = (int16_t)round(outputFromDSP->pointCloudOut.pointCloud[pntIdx].velocity * gMmwMssMCB.pointUnitInv.dopplerUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].snr =       (int16_t)outputFromDSP->pointCloudOut.snr[pntIdx].snr; //snr is in dB in Q8 format
        // gMmwMssMCB.pointCloudToUart.point[pntIdx].noise = outputFromDSP->pointCloudOut.snr[pntIdx].noise; //snr is in dB in Q8 format

        // Tracker DPU
        if (gMmwMssMCB.trackerCfg.staticCfg.trackerEnabled)
        {
        }
        

        // gMmwMssMCB.pointCloudToFeatExtr[pntIdx].vectorSph.elev    = outputFromDSP->pointCloudOut.pointCloud[pntIdx].elevAngle;
        // gMmwMssMCB.pointCloudToFeatExtr[pntIdx].vectorSph.range   = outputFromDSP->pointCloudOut.pointCloud[pntIdx].range;
        // gMmwMssMCB.pointCloudToFeatExtr[pntIdx].doppler        = outputFromDSP->pointCloudOut.pointCloud[pntIdx].velocity;
        // gMmwMssMCB.pointCloudToFeatExtr[pntIdx].snr            = (float)outputFromDSP->pointCloudOut.snr[pntIdx].snr * gMmwMssMCB.pointCloudToUart.pointUnit.snrUint;
    }
}

int32_t MmwDemo_startDemoProcessing(void)
{
    int32_t errCode = 0;
    int32_t retVal = SystemP_SUCCESS;

    if (gMmwMssMCB.adcDataSourceCfg.source == 0)
    {
        /* Populate all the control and adcbuff configurations */
        MmwDemo_populateControlCfg();

        if((gMmwMssMCB.mmWaveCfg.calibCfg.restoreEnable == 1U) && (gMmwMssMCB.mmWaveCfg.initCfg.iswarmstart == FALSE))
        {
            /* Restore factory Calibration Data. */
            retVal = mmwDemo_factoryCal();
            if(retVal != SystemP_SUCCESS)
            {
                CLI_write ("Error: Factory calibration failed\r\n");
                retVal = SystemP_FAILURE;
                MmwDemo_debugAssert (0);
            }
        }
	    /* If ADC logging via LVDS is enabled, Configure LVDS streaming parameters */
	    if(gMmwMssMCB.adcLogging.enable == 1)
	    {
	        /* NOTE: When LVDS streaming is configured, make sure ADPLL is set to 1600MHz and HS_DIVIDER_CLKOUT2 is enabled */
	        MmwDemo_configLVDSData();
	    }

        /* FECSS/APLL Clock Turn ON */
        retVal = MMWave_FecssDevClockCtrl(&gMmwMssMCB.mmWaveCfg.initCfg, &errCode);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            CLI_write ("Error: FECSS/APLL Clock Turn ON failed\r\n");
            retVal = SystemP_FAILURE;
            MmwDemo_debugAssert (0);
        }

        /* FECSS RF Power ON*/
        retVal = MMWave_FecssRfPwrOnOff(gMmwMssMCB.mmWaveCfg.txEnbl, gMmwMssMCB.mmWaveCfg.rxEnbl, &errCode);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            CLI_write ("Error: FECSS RF Power ON/OFF failed\r\n");
            retVal = SystemP_FAILURE;
            MmwDemo_debugAssert (0);
        }

        if((gMmwMssMCB.mmWaveCfg.calibCfg.restoreEnable != 1U) && (gMmwMssMCB.mmWaveCfg.initCfg.iswarmstart == FALSE))
        {
            /* Perform factory Calibrations. */
            retVal = mmwDemo_factoryCal();
            if(retVal != SystemP_SUCCESS)
            {
                CLI_write ("Error: mmWave factory calibration failed\r\n");
                retVal = SystemP_FAILURE;
                MmwDemo_debugAssert (0);
            }
        }
    }

    gIsSensorStarted = 1;
    
    if (gMmwMssMCB.adcDataSourceCfg.source == 0)
    {
        if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmWaveCfg, &errCode) < 0)
        {
            CLI_write ("Error: mmWave open failed [Error code %d]\n", errCode);
            retVal = SystemP_FAILURE;
            goto exit;
        }

        if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmWaveCfg, &errCode) < 0)
        {
            CLI_write ("Error: mmWave config failed [Error code %d]\n", errCode);
            retVal = SystemP_FAILURE;
            goto exit;
        }
    }

    if (gMmwMssMCB.oneTimeConfigDone)
    {
        gMmwMssMCB.frmCntrInSlidingWindowInitVal++;
        if (gMmwMssMCB.frmCntrInSlidingWindowInitVal == gMmwMssMCB.sigProcChainCommonCfg.numFrmPerSlidingWindow)
        {
            gMmwMssMCB.frmCntrInSlidingWindowInitVal = 0;
        }
    }
    else
    {
        gMmwMssMCB.frmCntrInSlidingWindowInitVal = 0;
    }

    gMmwMssMCB.frmCntrInSlidingWindowUart = 0;

    gDpcTask = xTaskCreateStatic(MmwDemo_dpcTask, /* Pointer to the function that implements the task. */
                                 "dpc_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                 DPC_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                 NULL,                  /* We are not using the task parameter. */
                                 DPC_TASK_PRI,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                 gDpcTaskStack,      /* pointer to stack base */
                                 &gDpcTaskObj);         /* pointer to statically allocated task object memory */
    configASSERT(gDpcTask != NULL);


    gTlvTask = xTaskCreateStatic(MmwDemo_transmitProcessedOutputTask, /* Pointer to the function that implements the task. */
                                 "tlv_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                 TLV_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                 NULL,                  /* We are not using the task parameter. */
                                 TLV_TASK_PRI,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                 gTlvTaskStack,      /* pointer to stack base */
                                 &gTlvTaskObj);         /* pointer to statically allocated task object memory */
    configASSERT(gTlvTask != NULL);

    SemaphoreP_pend(&gMmwMssMCB.dpcTaskConfigDoneSemHandle, SystemP_WAIT_FOREVER);
    SemaphoreP_pend(&gMmwMssMCB.uartTaskConfigDoneSemHandle, SystemP_WAIT_FOREVER);

    if (gMmwMssMCB.adcDataSourceCfg.source == 0)
    {
        if (gMmwMssMCB.oneTimeConfigDone)
        {
            /* Low Power mode latency End time */
            ll_LPmode_LatencyEnd = PRCMSlowClkCtrGet();
            gMmwMssMCB.stats.d_LPmode_Latency_us = ((ll_LPmode_LatencyEnd - ll_LPmode_LatencyStart) * M_TICKS_TO_USEC_SLOWCLK) - (double)gMmwMssMCB.stats.ll_FrameIdleTime_us;
        }

        if (MMWave_start (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmWaveCfg.strtCfg, &errCode) < 0)
        {
            /* Error/Warning: Unable to start the mmWave module */
            CLI_write ("Error: mmWave Start failed [Error code %d]\n", errCode);
            /* datapath has already been moved to start state; so either we initiate a cleanup of start sequence or
            assert here and re-start from the beginning. For now, choosing the latter path */
            MmwDemo_debugAssert(0);
            retVal = SystemP_FAILURE;
            goto exit;
        }
    }
    else
    {
        if (!gMmwMssMCB.oneTimeConfigDone)
        {
            gAdcFileTask = xTaskCreateStatic(MmwDemo_adcFileReadTask, /* Pointer to the function that implements the task. */
                                     "adcFileRead_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                     ADC_FILEREAD_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                     NULL,                  /* We are not using the task parameter. */
                                     ADC_FILEREAD_TASK_PRI,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                     gAdcFileTaskStack,      /* pointer to stack base */
                                     &gAdcFileTaskObj);         /* pointer to statically allocated task object memory */
            configASSERT(gAdcFileTask != NULL);
        }
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        gMmwMssMCB.oneTimeConfigDone = 1;
    }

exit:
    return errCode;
}

int32_t app_ioWrite(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, const uint8_t *txBuf)
{
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t writeBuf[3U] = {0U};

    // Parameter check
    if ((pmicHandle == NULL) || (txBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && ((bufLen == 0U) || (bufLen > 2U)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // writeBuf[0U]: Target device internal register address
        // writeBuf[1U] and onwards: txBuf
        writeBuf[0U] = regAddr;
        memcpy(&(writeBuf[1U]), txBuf, bufLen);

        // Initialize I2C handle and I2C transaction struct
        i2cHandle = *((I2C_Handle *)pmicHandle->commHandle);
        I2C_Transaction_init(&i2cTransaction);

        /*** Configure I2C transaction for a write ***/
        i2cTransaction.targetAddress = pmicHandle->i2cAddr;
        i2cTransaction.writeBuf = writeBuf;
        i2cTransaction.writeCount = bufLen + 1U;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.readCount = 0U;

        // Initiate write
        status = I2C_transfer(i2cHandle, &i2cTransaction);

        // Convert platform-specific success/error code to driver success/error code
        if (status != I2C_STS_SUCCESS)
        {
            status = PMIC_ST_ERR_I2C_COMM_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    return status;
}

int32_t app_ioRead(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t bufLen, uint8_t *rxBuf)
{
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;
    int32_t status = PMIC_ST_SUCCESS;

    // Parameter check
    if ((pmicHandle == NULL) || (rxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (bufLen == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Initialize I2C handle and I2C transaction struct
        i2cHandle = *((I2C_Handle *)pmicHandle->commHandle);
        I2C_Transaction_init(&i2cTransaction);

        /*** Configure I2C transaction for a read ***/
        i2cTransaction.targetAddress = pmicHandle->i2cAddr;
        i2cTransaction.writeBuf = &regAddr;
        i2cTransaction.writeCount = 1U;
        i2cTransaction.readBuf = rxBuf;
        i2cTransaction.readCount = bufLen;

        // Initiate read
        status = I2C_transfer(i2cHandle, &i2cTransaction);

        // Convert platform-specific success/error code to driver success/error code
        if (status != I2C_STS_SUCCESS)
        {
            status = PMIC_ST_ERR_I2C_COMM_FAIL;
        }
        else
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    return status;
}


void people_tracking_6844_mss(void* args)
{
    int32_t errorCode = SystemP_SUCCESS;
    int32_t retVal = -1;

    /* Peripheral Driver Initialization */
    Drivers_open();
    Board_driversOpen();

    I2C_Handle  i2cHandle = gI2cHandle[CONFIG_I2C0];
    
    /* Configuring the INA sensor for power measurement */
    SensorConfig(i2cHandle);


    Pmic_CoreCfg_t pmicCfg = {
            .i2cAddr = PMIC_CONFIG0_I2C_ADDRESS,
            .commHandle = &(gI2cHandle[CONFIG_I2C0]),
            .ioRead = &app_ioRead,
            .ioWrite = &app_ioWrite,
            .critSecStart = &app_critSecStart,
            .critSecStop = &app_critSecStop
        };
    int32_t status = PMIC_ST_SUCCESS;

    
    status = Pmic_init(&pmicCfg, &pmicHandle);
    if (status == PMIC_ST_SUCCESS)
    {
        printf("PMIC Initialized \n");
    }

    /* Disable watchdog timer */
    {
        uint8_t txBuf1[1];
        uint8_t txBuf2[1];
        uint8_t txBuf3[1];
        txBuf1[0] = 0x04;
        txBuf2[0] = 0x02;
        txBuf3[0] = 0x00;
        app_ioWrite(&pmicHandle,0x10,0x01, txBuf1);  // put watchdog in long window mode
        app_ioWrite(&pmicHandle,0x0F,0x01, txBuf2);  // deactivate watchdog
        app_ioWrite(&pmicHandle,0x10,0x01, txBuf3);  // disable watchdog long window mode
    }

    /* Unhalt DSP */
    SOC_rcmUnhaltDsp();

    // // Get the version of Device.
    // pgVersion = SOC_getEfusePgVersion();

    // Configure the LED GPIO
    gGpioBaseAddrLed = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    gPinNumLed       = GPIO_LED_PIN;
    GPIO_setDirMode(gGpioBaseAddrLed, gPinNumLed, GPIO_LED_DIR);

    /*HWASS_SHRD_RAM, TPCCA and TPCCB memory have to be init before use. */
    /*APPSS SHRAM0 and APPSS SHRAM1 memory have to be init before use. However, for awrL varients these are initialized by RBL */
    /*FECSS SHRAM (96KB) has to be initialized before use as RBL does not perform initialization.*/
    SOC_memoryInit(SOC_MEMINIT_APPSS_SHARED_TCMA_BANK0_INIT|SOC_MEMINIT_APPSS_SHARED_TCMA_BANK1_INIT|SOC_MEMINIT_APPSS_SHARED_TCMB_INIT|SOC_MEMINIT_FECSS_SHARED_RAM_INIT|SOC_MEMINIT_DSS_L3_NATIVE_RAM0_INIT|SOC_MEMINIT_DSS_L3_NATIVE_RAM1_INIT|SOC_MEMINIT_APPSS_TPCC_INIT|SOC_MEMINIT_DSS_TPCC_INIT);

    gMmwMssMCB.commandUartHandle = gUartHandle[0];
    gMmwMssMCB.loggingUartHandle = gUartHandle[1];
    
	/* EDMA handle*/
    gMmwMssMCB.edmaHandle = gEdmaHandle[CONFIG_EDMA0];
	gMmwMssMCB.edmaHandle1 = gEdmaHandle[CONFIG_EDMA0];

    /* mmWave initialization*/
    MmwDemo_mmWaveInit(0);

    /* Initialize default antenna geometry */
    memcpy((void *) &gMmwMssMCB.antennaGeometryCfg, (void *) &gDefaultAntGeometry, sizeof(MmwDemo_antennaGeometryCfg));

    /*TODO - Add back below 2 tasks after Power dependencies available*/
    gMmwInitTask = xTaskCreateStatic( mmwreinitTask,      /* Pointer to the function that implements the task. */
                                   "mmwinit",          /* Text name for the task.  This is to facilitate debugging only. */
                                   MMWINIT_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                   NULL,            /* We are not using the task parameter. */
                                   MMWINITTASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                   gMmwInitTaskStack,  /* pointer to stack base */
                                   &gMmwInitTaskObj ); /* pointer to statically allocated task object memory */
    gMmwInit = xSemaphoreCreateBinaryStatic(&gMmwInitObj);

    // Radar Power Management Framework: Create a Task for Power Management Framework
    gPowerTask = xTaskCreateStatic( powerManagementTask,      /* Pointer to the function that implements the task. */
                                   "power",          /* Text name for the task.  This is to facilitate debugging only. */
                                   POWER_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                   NULL,            /* We are not using the task parameter. */
                                   POWER_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                   gPowerTaskStack,  /* pointer to stack base */
                                   &gPowerTaskObj ); /* pointer to statically allocated task object memory */
                                  
    // //Radar Power Management Framework: Create Semaphore for to pend Power Task
    gPowerSem = xSemaphoreCreateBinaryStatic(&gPowerSemObj);

    /* Create binary semaphores to pend at different stages of the OOB */
    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.demoInitTaskCompleteSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.cliInitTaskCompleteSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.TestSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.tlvSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.adcFileTaskSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.dpcTaskConfigDoneSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.uartTaskConfigDoneSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.lvdsStream.frameDoneSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    /* Initialize Flash interface. */
    retVal = MmwDemo_flashInit();
    if (retVal < 0)
    {
        CLI_write("Error: Flash Initialization Failed!\r\n");
        MmwDemo_debugAssert (0);
    }

    /* Check if the device is RF-Trimmed */
    /* Checking one Trim is enough */
    // if(SOC_rcmReadSynthTrimValid() == RF_SYNTH_TRIM_VALID)
    // {
    //     gMmwMssMCB.factoryCalCfg.atecalibinEfuse = true;
    // }
    // else
    // {
    //     gMmwMssMCB.factoryCalCfg.atecalibinEfuse = false;
    //     CLI_write("Error: Device is not RF-Trimmed!\r\n");
    //     MmwDemo_debugAssert (0);
    // }

    /* Initialize LVDS streaming components */
    if ((errorCode = MmwDemo_LVDSStreamInit()) < 0 )
    {
        CLI_write("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n",errorCode);
        return;
    }

    /* DPC initialization*/
    DPC_Init();

    // Make the LED HIGH
    GPIO_pinWriteHigh(gGpioBaseAddrLed, gPinNumLed);

    CLI_init(CLI_TASK_PRIORITY);

    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.demoInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);

    Board_driversClose();
    Drivers_close();
}
