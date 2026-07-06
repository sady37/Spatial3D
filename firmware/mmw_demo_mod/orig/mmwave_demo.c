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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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

#include <source/power_management/power_management.h>
#include <source/mmw_res.h>
#include <source/mmw_cli.h>
#include <source/mmwave_demo.h>
#include <source/calibrations/mmw_flash_cal.h>
#include <source/dpc/dpc.h>
#include <source/calibrations/range_phase_bias_measurement.h>
#include <source/lvds_streaming/mmw_lvds_stream.h>
#include <source/calibrations/factory_cal.h>

#include <ti_drivers_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <ti_board_config.h>

/**************************************************************************
 *************************** Macros Definitions ***************************
 **************************************************************************/

/*! Local RAM buffer size for object detection DPC */
#define MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE ((8U+6U+4U+2U+8U) * 1024U)
/* Based on practical measurements conducted on different block sizes, this is the time needed(in us) to transmit a single byte of data */
#define UART_BYTE_TRANSMIT_TIME1250000_US  9   

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
uint8_t                         gMmwL3[L3_MEM_SIZE]  __attribute((section(".bss.l3")));

/*! Local RAM buffer for object detection DPC */
uint8_t                         gMmwCoreLocMem[MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE];

/*! HWA driver instance handle */
HWA_Handle                      gHwaHandle;

/*! Temperature stats info */
MMWave_temperatureStats         gTempStats;

/*! LED configurations */
uint32_t                        gGpioBaseAddrLed, gPinNumLed;

/*! SPI Host Intr configurations */
uint32_t                        gSPIHostIntrBaseAddrLed,gSPIHostIntrPinNumLed;

/* For Sensor Stop */
uint32_t                        gSensorStop = 0;

float                           gSocClk = 40000000; //Hz

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

/* Buffer to store Raw ADC Data per frame */
uint8_t adcbuffer[ADC_DATA_BUFF_MAX_SIZE] = {0};
/* Number of bytes in every frame */
uint32_t adcDataPerFrame;

/* For freeing the channels after Sensor Stop */
void MmwDemo_freeDmaChannels(EDMA_Handle edmaHandle);

void MmwDemo_transmitProcessedOutputTask();

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern uint8_t                       gIsSensorStarted;
extern TaskHandle_t                 gDpcTask;
extern TaskHandle_t                 gAdcFileTask;

extern void MmwDemo_populateControlCfg();
extern void MmwDemo_dpcTask();
extern void MmwDemo_adcFileReadTask();

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

/**
 *  @b Description
 *  @n
 *      Parks the LVDS pins
 */
void mmWDemo_parkLvdsPins()
{
    /* Parking LVDS Pins */
    HW_WR_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL0, 0x39393939);
    HW_WR_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL1, 0x01003939);
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
 *      MMW Demo helper Function to Stop the Sensor. Sensor Stop in honoured only when Low Power Mode is disabled.
 *
 *  @retval
 *      None
 */
void MmwDemo_stopSensor(void)
{
    int32_t err;
    // Stop and Close the front end
    MMWave_stop(gMmwMssMCB.ctrlHandle,&err);
    MMWave_close(gMmwMssMCB.ctrlHandle,&err);
    if (gMmwMssMCB.adcLogging.enable == ADC_DATA_LOGGING_LVDS_STREAMING)
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

    // Delete the DPC, TLV as we will create them again in next configuration when we start
    vTaskDelete(gDpcTask);
    vTaskDelete(NULL);
}

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern void MmwDemo_ADCBufConfig(uint16_t rxChannelEn, uint32_t chanDataSize);

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

    if(!iswarmstrt)
    {
        /* Initialize the mmWave control init configuration */
        memset ((void*)&gMmwMssMCB.mmWaveCfg, 0, sizeof(MMWave_Cfg));
    }
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
void MmwDemo_transmitProcessedOutputTask()
{
    UART_Handle uartHandle = gMmwMssMCB.loggingUartHandle;
    I2C_Handle  i2cHandle = gI2cHandle[CONFIG_I2C0];
    DPC_ObjectDetection_ExecuteResult *result = &gMmwMssMCB.dpcResult;
    MmwDemo_output_message_header header;
    CLI_GuiMonSel   *pGuiMonSel;
    uint32_t tlvIdx = 0;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    int32_t uartTransferStartTimeStampUs, totalTimeElapsedUs, computedUARTTransmitTimeUs = 0, maxUARTTransferTimeAvailableUs = 0;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_ALL_MSG_MAX];

    /* Get Gui Monitor configuration */
    pGuiMonSel = &gMmwMssMCB.guiMonSel;

    /* Send signal to CLI task that this task is ready */
    SemaphoreP_post(&gMmwMssMCB.uartTaskConfigDoneSemHandle);

    while(true)
    {
        SemaphoreP_pend(&gMmwMssMCB.tlvSemHandle, SystemP_WAIT_FOREVER);
        tlvIdx = 0;

        /* Clear message header */
        memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
        /* Header: */
        header.platform =  0xA6844;
        header.magicWord[0] = 0x0102;
        header.magicWord[1] = 0x0304;
        header.magicWord[2] = 0x0506;
        header.magicWord[3] = 0x0708;
        header.numDetectedObj = result->numObjOut;
        header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                            (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                            (MMWAVE_SDK_VERSION_MINOR << 16) |
                            (MMWAVE_SDK_VERSION_MAJOR << 24);



        packetLen = sizeof(MmwDemo_output_message_header);
        if ((pGuiMonSel->pointCloud == 1) && (result->numObjOut > 0))
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
            tl[tlvIdx].length = sizeof(DPIF_PointCloudCartesian) * result->numObjOut;
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /* Side info */
        if ((pGuiMonSel->pointCloud == 1) && result->numObjOut > 0)
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO;
            tl[tlvIdx].length = sizeof(DPIF_PointCloudSideInfo) * result->numObjOut;
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /* Range Profile */
        if ((pGuiMonSel->rangeProfile & 0x1))
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
            tl[tlvIdx].length = sizeof(uint16_t) * gMmwMssMCB.numRangeBins;
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }
        
        /* Range-Doppler Heatmap */
        if ((pGuiMonSel->rangeDopplerHeatMap) && (result->rngDopplerHeatMap != NULL))
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
            tl[tlvIdx].length = gMmwMssMCB.numRangeBins * gMmwMssMCB.numDopplerBins * sizeof(uint16_t);
            packetLen += sizeof(MmwDemo_output_message_tl) +  tl[tlvIdx].length;
            tlvIdx++;
        }

        if (pGuiMonSel->statsInfo)
        {
            /* Initialize the tempReading configuration to 0 */
            memset ((void*)&gMmwMssMCB.outStats.tempReading[0], 0, (4 * sizeof(int16_t)));
            for(int i=0; i<4; i++)
            {
                gMmwMssMCB.outStats.tempReading[0]+=gTempStats.tempValue[i];
                gMmwMssMCB.outStats.tempReading[1]+=gTempStats.tempValue[i+4];
            }
            gMmwMssMCB.outStats.tempReading[0] = gMmwMssMCB.outStats.tempReading[0]/4; // Average of all Rx temp
            gMmwMssMCB.outStats.tempReading[1] = gMmwMssMCB.outStats.tempReading[1]/4; // Average of all Tx temp
            gMmwMssMCB.outStats.tempReading[2] = gTempStats.tempValue[8]; // PM temp
            gMmwMssMCB.outStats.tempReading[3] = gTempStats.tempValue[9]; // DIG temp
            mmwDemo_PowerMeasurement(i2cHandle,&gMmwMssMCB.outStats.powerMeasured[0]);
            tl[tlvIdx].type = MMWDEMO_OUTPUT_EXT_MSG_STATS;
            tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
            packetLen += sizeof(MmwDemo_output_message_tl) +  tl[tlvIdx].length;
            tlvIdx++;
        }

        if (gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
        {
            tl[tlvIdx].type = MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO;
            tl[tlvIdx].length = sizeof(DPIF_compRxChannelBiasFloatCfg);
            packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
            tlvIdx++;
        }

        /* Fill header */
        header.numTLVs = tlvIdx;
        /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
        header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
                ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
        header.frameNumber = gMmwMssMCB.stats.frameStartIntCounter; 

        /* Reserved fields */
        header.timeCpuCycles =  0;
        header.subFrameNumber = -1;

        if(gUartParams[1].baudRate == 1250000)
        {
            uartTransferStartTimeStampUs = ClockP_getTimeUsec();
            totalTimeElapsedUs = uartTransferStartTimeStampUs - gMmwMssMCB.stats.frameStartTimeStampUs;
            computedUARTTransmitTimeUs = header.totalPacketLen * UART_BYTE_TRANSMIT_TIME1250000_US;
            maxUARTTransferTimeAvailableUs = (int32_t)gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus - totalTimeElapsedUs;

            if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
            {   
                /* The remaining time, calculated as (gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus - totalTimeElapsedUs), should be sufficient to accommodate the minimum deep sleep duration and the time needed for TLV transmission. 
                * Since IDLE mode has the lowest threshold, it is subtracted from (gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus - totalTimeElapsedUs) to determine the maximum available time for TLV transmission. */
                int32_t idleModeThresholdUs = Power_getThresholds(POWER_IDLE);
                maxUARTTransferTimeAvailableUs = maxUARTTransferTimeAvailableUs - idleModeThresholdUs;
            }
            if(computedUARTTransmitTimeUs >= maxUARTTransferTimeAvailableUs)
            {
                CLI_write ("\r\n Warning: Frame Time is not enough to transfer all the configured TLVs!!! \r\n");
            }
        }
        if(gUartParams[1].baudRate != 1250000 || ((gUartParams[1].baudRate == 1250000) && (computedUARTTransmitTimeUs < maxUARTTransferTimeAvailableUs)))
        {
            /* Processing finished start UART transmission */
            if(tlvIdx != 0)
            {
                MmwDemo_uartWrite (uartHandle, (uint8_t*)&header, sizeof(MmwDemo_output_message_header));
                tlvIdx = 0;
            }

            /* Send detected Objects */
            if ((pGuiMonSel->pointCloud == 1) && (result->numObjOut > 0))
            {
                MmwDemo_uartWrite (uartHandle,
                                (uint8_t*)&tl[tlvIdx],
                                sizeof(MmwDemo_output_message_tl));

                /*Send array of objects */
                MmwDemo_uartWrite (uartHandle, (uint8_t*)result->objOut,
                                sizeof(DPIF_PointCloudCartesian) * result->numObjOut);
                tlvIdx++;
            }

            /* Send detected Objects Side Info */
            if ((pGuiMonSel->pointCloud == 1) && (result->numObjOut > 0))
            {
                MmwDemo_uartWrite (uartHandle,
                                (uint8_t*)&tl[tlvIdx],
                                sizeof(MmwDemo_output_message_tl));

                /*Send array of objects */
                MmwDemo_uartWrite (uartHandle, (uint8_t*)result->objOutSideInfo,
                                sizeof(DPIF_PointCloudSideInfo) * result->numObjOut);
                tlvIdx++;
            }

            /* Send Range profile */
            if ((pGuiMonSel->rangeProfile & 0x1))
            {
                MmwDemo_uartWrite (uartHandle,
                                (uint8_t*)&tl[tlvIdx],
                                sizeof(MmwDemo_output_message_tl));

                for(uint16_t i = 0; i < gMmwMssMCB.numRangeBins; i++)
                {
                    MmwDemo_uartWrite (uartHandle,
                            (uint8_t*)&gMmwMssMCB.detMatrix[i * gMmwMssMCB.numDopplerBins],
                            sizeof(uint16_t));
                }
                tlvIdx++;
            }
            
            /* Send Range-Doppler Heatmap */
            if ((pGuiMonSel->rangeDopplerHeatMap) && (result->rngDopplerHeatMap != NULL))
            {
                MmwDemo_uartWrite (uartHandle,
                                (uint8_t*)&tl[tlvIdx],
                                sizeof(MmwDemo_output_message_tl));

                {
                    int ii;
                    for (ii=0; ii<gMmwMssMCB.numRangeBins; ii++)
                    {
                        MmwDemo_uartWrite (uartHandle,
                                (uint8_t *) &result->rngDopplerHeatMap[ii*gMmwMssMCB.numDopplerBins],
                                gMmwMssMCB.numDopplerBins * sizeof(uint16_t));
                    }
                }

                tlvIdx++;
            }

            /* Send stats information (interframe processing time and uart transfer time) */
            if (pGuiMonSel->statsInfo)
            {
                MmwDemo_uartWrite (uartHandle,
                                (uint8_t*)&tl[tlvIdx],
                                sizeof(MmwDemo_output_message_tl));

                MmwDemo_uartWrite (uartHandle,
                            (uint8_t*) &gMmwMssMCB.outStats,
                            tl[tlvIdx].length);
                tlvIdx++;
            }

            /* Send Rx Channel compensation coefficients */
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

            if(tlvIdx != 0)
            {
                /* Send padding bytes */
                numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
                if (numPaddingBytes < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
                {
                    MmwDemo_uartWrite (uartHandle, (uint8_t*)padding, numPaddingBytes);
                }
            }
        }

        /* Flush UART buffer here for each frame. */
        UART_flushTxFifo(uartHandle);

        /* End of UART data transmission */
        gMmwMssMCB.stats.uartTransferEndTimeStampUs = ClockP_getTimeUsec();
        gMmwMssMCB.outStats.transmitOutputTimeUs = (gMmwMssMCB.stats.uartTransferEndTimeStampUs - gMmwMssMCB.stats.ProcessingEndTimeStampUs);
        
        /* Interframe processing and UART data transmission completed */
        gMmwMssMCB.interSubFrameProcToken--;

        /* Capture the Demo Active and Idle time in Low Power Disabled mode */
        if (gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_DISABLE)
        {
            gMmwMssMCB.stats.totalActiveTimeUs = (gMmwMssMCB.stats.uartTransferEndTimeStampUs - gMmwMssMCB.stats.frameStartTimeStampUs);
            gMmwMssMCB.stats.ll_FrameIdleTimeus = (unsigned long long)(gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus - gMmwMssMCB.stats.totalActiveTimeUs);
            if(gMmwMssMCB.adcDataSourceCfg.source == 1)
            {
                /* In offline ADC injection mode, the total active time (totalActiveTimeUs) is irrelevant; therefore, it is being set to 0 */
                gMmwMssMCB.stats.totalActiveTimeUs = 0;
                /* In test mode trigger next frame processing */
                SemaphoreP_post(&gMmwMssMCB.adcFileTaskSemHandle);
            }

            /* The Sensor Stop command is executed if Low Power Configuration is disabled or after completing all configured frames. */
            if((gSensorStop == 1) || ((gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames != 0) && (gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames == gMmwMssMCB.stats.frameStartIntCounter)))
            {
                MmwDemo_stopSensor();
            }
        }
 
        if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
        {
            xSemaphoreGive(gPowerSem);
            if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
            {
                Power_enablePolicy();
            }
        }
    }
}

void MmwDemo_AdcBufPerFrmEDMAConfig()
{
    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    uint8_t             *srcBuffPtr, *dstBuffPtr;
    EDMACCPaRAMEntry    edmaParam;
    uint32_t            dmaCh, tcc, param, parambackup;
    
    /* Allocate memory for buffer */
    adcDataPerFrame = gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * gMmwMssMCB.numRxAntennas * 2;
    /* Configure EDMA channel for reading the Raw ADC data */
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[CONFIG_EDMA1]);
    regionId = EDMA_getRegionId(gEdmaHandle[CONFIG_EDMA1]);
    dmaCh = EDMA_APPSS_TPCC_A_CHIRP_AVAIL_IRQ;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[CONFIG_EDMA1], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    tcc = EDMA_APPSS_TPCC_A_CHIRP_AVAIL_IRQ;
    testStatus = EDMA_allocTcc(gEdmaHandle[CONFIG_EDMA1], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    param = EDMA_APPSS_TPCC_A_CHIRP_AVAIL_IRQ;
    testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA1], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    parambackup = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA1], &parambackup);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    srcBuffPtr = (uint8_t *) CSL_DSS_ADCBUF_READ_U_BASE;
    dstBuffPtr = (uint8_t *) SOC_virtToPhy(&adcbuffer);
    /* Request channel */
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
    dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);
    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * gMmwMssMCB.numRxAntennas * 2;
    edmaParam.bCnt          = (uint16_t) gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame;
    edmaParam.srcBIdx       = (int16_t) 0;
    edmaParam.destBIdx      = (int16_t) (gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * gMmwMssMCB.numRxAntennas * 2);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) (-1 * adcDataPerFrame);
    edmaParam.linkAddr      = (0x4000 + (32 * parambackup));
    edmaParam.opt          |= ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);
    EDMASetPaRAM(baseAddr, param, &edmaParam);
    EDMAEnableTransferRegion(baseAddr, regionId, dmaCh,
        EDMA_TRIG_MODE_EVENT);

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * gMmwMssMCB.numRxAntennas * 2;
    edmaParam.bCnt          = (uint16_t) gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame;
    edmaParam.srcBIdx       = (int16_t) 0;
    edmaParam.destBIdx      = (int16_t) (gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * gMmwMssMCB.numRxAntennas * 2);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) (-1 * adcDataPerFrame);
    edmaParam.linkAddr      = (0x4000 + (32 * parambackup));
    edmaParam.opt          |= ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);
    EDMASetPaRAM(baseAddr, parambackup, &edmaParam);
}

int32_t MmwStart(void)
{
    int32_t errCode = 0;
    int32_t retVal = SystemP_SUCCESS;
    int32_t statEnable;
    
    /* Populate all ADC Buff Configs. */
    MmwDemo_ADCBufConfig(gMmwMssMCB.mmWaveCfg.rxEnbl, (gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples*2));

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
    if(gMmwMssMCB.adcLogging.enable == ADC_DATA_LOGGING_LVDS_STREAMING)
    {
        /* Configure LVDS stream */
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

    /* FECSS RF Power ON/OFF for RF Channel Configs */
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
            gMmwMssMCB.stats.d_LPmode_Latencyus = ((ll_LPmode_LatencyEnd - ll_LPmode_LatencyStart) * M_TICKS_TO_USEC_SLOWCLK) - (double)gMmwMssMCB.stats.ll_FrameIdleTimeus;
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

        if(gMmwMssMCB.gpAdcCfg.channelEnable != 0)
        {
            /* Enable GPADC channels based on the gpAdcCfg CLI command */
            statEnable = MMWave_enableGPADC(gMmwMssMCB.gpAdcCfg.channelEnable);

            if(statEnable != 0)
            {
                CLI_write("\r\n GPADC Config Error : %d \r\n",statEnable);
            }
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

    
    if(gMmwMssMCB.adcLogging.enable == ADC_DATA_LOGGING_SPI_STREAMING)
    {   
        MmwDemo_AdcBufPerFrmEDMAConfig();
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        gMmwMssMCB.oneTimeConfigDone = 1;
    }

exit:
    return errCode;
}

void mmwave_demo(void* args)
{
    int32_t errorCode = SystemP_SUCCESS;
    int32_t retVal = -1;

    /* Peripheral Driver Initialization */
    Drivers_open();
    Board_driversOpen();

    I2C_Handle  i2cHandle = gI2cHandle[CONFIG_I2C0];

    // Configure the LED GPIO
    gGpioBaseAddrLed = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    gPinNumLed       = GPIO_LED_PIN;
    GPIO_setDirMode(gGpioBaseAddrLed, gPinNumLed, GPIO_LED_DIR);

    // Configure the FTDI HOST INTR PIN
    gSPIHostIntrBaseAddrLed = (uint32_t) AddrTranslateP_getLocalAddr(SPI_HOST_INTR_BASE_ADDR);
    gSPIHostIntrPinNumLed       = SPI_HOST_INTR_PIN;
    GPIO_setDirMode(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed, SPI_HOST_INTR_DIR);


    /* Configuring the INA sensor for power measurement */
    SensorConfig(i2cHandle);

    /*HWASS_SHRD_RAM, TPCCA and TPCCB memory have to be init before use. */
    /*APPSS SHRAM0 and APPSS SHRAM1 memory have to be init before use. However, for awrL varients these are initialized by RBL */
    /*FECSS SHRAM (96KB) has to be initialized before use as RBL does not perform initialization.*/
    SOC_memoryInit(SOC_MEMINIT_APPSS_SHARED_TCMA_BANK0_INIT|SOC_MEMINIT_APPSS_SHARED_TCMA_BANK1_INIT|SOC_MEMINIT_APPSS_SHARED_TCMB_INIT|SOC_MEMINIT_FECSS_SHARED_RAM_INIT|SOC_MEMINIT_DSS_L3_NATIVE_RAM0_INIT|SOC_MEMINIT_DSS_L3_NATIVE_RAM1_INIT|SOC_MEMINIT_APPSS_TPCC_INIT|SOC_MEMINIT_DSS_TPCC_INIT);

    /*CLI and TLV UART handles*/
    gMmwMssMCB.commandUartHandle = gUartHandle[0];
    gMmwMssMCB.loggingUartHandle = gUartHandle[1];

    /* EDMA handle*/ 
    gMmwMssMCB.edmaHandle = gEdmaHandle[CONFIG_EDMA0];

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
                                  
    //Radar Power Management Framework: Create Semaphore for to pend Power Task
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

    /*Setting the FTDI HOST INTR pin to default high state for proper behaviour of SPI ADC Streaming*/
    GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

    /* Initialize Flash interface. */
    retVal = MmwDemo_flashInit();
    if (retVal < 0)
    {
        CLI_write("Error: Flash Initialization Failed!\r\n");
        MmwDemo_debugAssert (0);
    }

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
    This is needed because the DCA1000EVM FPGA needs the delay to lock to the
    bit clock before they can start capturing the data correctly. */
    ClockP_usleep(12 * 1000);
    /* DPC initialization*/
    DPC_Init();

    CLI_init(CLI_TASK_PRIORITY);

    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.demoInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);

    Board_driversClose();
    Drivers_close();
}