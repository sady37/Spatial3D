/*
 * Copyright (C) 2022-24 Texas Instruments Incorporated
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

#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>

/* mmwave SDK files */
#include <control/mmwave/mmwave.h>
#include <source/mmw_cli.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <drivers/prcm.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/soc.h>

#include <source/power_management/power_management.h>
#include <source/mmwave_demo_mss.h>
#include <board/ina.h>
#include <drivers/power.h>
#include <drivers/power_xwrL68xx.h>
#include <source/dpc/dpc_mss.h>
#include <datapath/dpu/rangeproc/v1/rangeprochwa.h>
#include <datapath/dpu/rangeproc/v1/rangeprochwa_internal.h>


#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)

/* ----------- ClockP ----------- */
#define APPSS_RTI_CLOCK_SRC_MUX_ADDR (0x56040034u)
#define APPSS_RTI_CLOCK_SRC_OSC_CLK (0x0u)
#define APPSS_RTI_BASE_ADDR     (0x56F7F000u)

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern TaskHandle_t gCliTask;
extern int32_t MmwDemo_mmWaveInit(bool iswarmstrt);
extern int32_t MmwDemo_startDemoProcessing(void);
extern SemaphoreHandle_t gPowerSem;
//For Sensor Stop
extern uint32_t gSensorStop;
extern int8_t gIsSensorStarted;
// LED config
extern uint32_t gGpioBaseAddrLed, gPinNumLed;

extern TaskHandle_t gDpcTask;
extern TaskHandle_t gTlvTask;
extern int32_t MmwStart(void);

extern StaticTask_t gMmwInitTaskObj;
extern TaskHandle_t gMmwInitTask;
extern StaticSemaphore_t gMmwInitObj;
extern SemaphoreHandle_t gMmwInit;
extern HWA_Handle gHwaHandle;
extern Power_ModuleState Power_module;
extern Power_ConfigV1 Power_config;

Power_SleepState gDemoLowPwrStateTaken = POWER_NONE;

// Free all the allocated EDMA channels
void mmwDemo_freeDmaChannels(EDMA_Handle edmaHandle);
// Re-init Function Declarations
void PowerClock_init();
void Pinmux_init();
void QSPI_init();
void EDMA_init();
void HWA_init();
void Drivers_uartInit();
void TimerP_init();
void System_init();

/**
*  @b Description
*  @n
 *      This function is user configurable hook before entering LPDS
 *
 */
/* FOR DEBUGGING */
volatile uint32_t gPauseAtEntryHook = 0;

void power_LPDSentryhook(void)
{
    uint32_t regVal;

    while (gPauseAtEntryHook)
    {
        ;
    }
    // Anything to do before LPDS entry
    mmwDemo_freeDmaChannels(gEdmaHandle[0]);
    Drivers_edmaClose();
    EDMA_deinit();
    ADCBuf_close(gMmwMssMCB.adcBuffHandle);
    HWA_close(gHwaHandle);
    Board_driversClose();
    Drivers_close();

    IpcNotify_deInit();

    /* Gating Peripherals */
    /* Swtiching UART A to OSC CLk */
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_0_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_SRCSEL,0x0);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_0_CLKCTL, regVal);

    /* Swtiching UART B to OSC CLk */
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_SRCSEL,0x0);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL, regVal);



    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_QSPI_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_SRCSEL,0x0);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_QSPI_CLKCTL, regVal);

    /* Swtiching MCSPI to OSC CLk */
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_SPI_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_SRCSEL,0x0);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_SPI_CLKCTL, regVal);


    /*LVDS*/
    regVal = HW_RD_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL0);
    HW_SET_FIELD32(regVal, CSL_TOP_CTRL_LVDS_PAD_CTRL0_LVDS_PAD_CTRL0_CTRL,0x39393939);
    HW_WR_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL0, regVal);

    regVal = HW_RD_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL1);
    HW_SET_FIELD32(regVal, CSL_TOP_CTRL_LVDS_PAD_CTRL1_LVDS_PAD_CTRL1_CTLR,0x01003939);
    HW_WR_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL1, regVal);
}

volatile uint32_t gDbgResumeHook = 0;
/**
*  @b Description
*  @n
 *      This function is user configurable hook after exiting LPDS
 *
 */
void power_LPDSresumehook(void)
{
    volatile static uint8_t ledState = 0;

    while (gDbgResumeHook)
    {
        ;
    }
    /* initialize Hwi but keep interrupts disabled */
    HwiP_init();

    /* init debug log zones early */
    /* Debug log init */
    DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
    DebugP_logZoneEnable(DebugP_LOG_ZONE_WARN);

    /* set timer clock source */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_APPSS_RCM, 0);
    *(volatile uint32_t*)(APPSS_RTI_CLOCK_SRC_MUX_ADDR) = APPSS_RTI_CLOCK_SRC_OSC_CLK;
    CSL_app_rcmRegs *ptrAPPRCMRegs = (CSL_app_rcmRegs *)CSL_APP_RCM_U_BASE;
    CSL_REG32_FINS(&(ptrAPPRCMRegs->IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_RTI, 0x0U);
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_APPSS_RCM, 0);

    /* Enable interrupt handling */
    HwiP_enable();
    PowerClock_init();
    
    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());

    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    QSPI_init();
    EDMA_init();
    ADCBuf_init(SystemP_WAIT_FOREVER);
    HWA_init();
    I2C_init();
    Power_init();

    Drivers_uartInit();
    Drivers_open();
    Board_driversOpen();

    /*Clear the Radar Wakeup Status Register*/
    //PRCMClearWakeUpStatus();

    /*HWASS_SHRD_RAM, TPCCA and TPCCB memory have to be init before use. */
    /*APPSS SHRAM0 and APPSS SHRAM1 memory have to be init before use. */
    /*FECSS SHRAM (96KB) has to be initialized before use */
    /* DPC initialization */
    DPC_Init();

    // Toggle the LED indicating out of deep sleep
    if(ledState == 0)
    {
        ledState = 1;
        GPIO_pinWriteLow(gGpioBaseAddrLed, gPinNumLed);
    }
    else
    {
       ledState = 0;
       GPIO_pinWriteHigh(gGpioBaseAddrLed, gPinNumLed);
    }
}

/**
*  @b Description
*  @n
*      Task to parallelize MMWave initialization and DPC init during LPDS exit re-init sequence
*
*/
void mmwreinitTask(void *args)
{
    while(1)
    {
        /* when the gMmwInit semaphore is given, initialize mmwave on warm reset  */
        xSemaphoreTake(gMmwInit, portMAX_DELAY);
        MmwDemo_mmWaveInit(1);
    }
}

volatile uint32_t gPauseAfterPowerUp = 0;
volatile uint32_t gWakeUpSrcMask = 0;
volatile uint32_t gPauseAfterUartWakeUp = 0;
volatile CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;

/**
*  @b Description
*  @n
 *      This function is task for Radar Power Management Framework
 *
 */
/* Radar Power Management Framework: Power Management Task */
volatile uint32_t gPauseEntryPowerTask = 0;
void powerManagementTask(void *args)
{
    uint64_t l_cfgframePeriodus, l_demoEndTimeUs;
    while(1)
    {
        /* Wait until the UART transmit is complete. Once UART data (if any) is transmitted, get to low power state */
        xSemaphoreTake(gPowerSem, portMAX_DELAY);

        while (gPauseEntryPowerTask)
        {
            ;
        }

        /* Delete the DPC task : We are recreating this task during exit of Low Power mode */
        vTaskDelete(gDpcTask);
        /* Delete the UART Tx task : We are recreating this task during exit of Low Power mode*/
        vTaskDelete(gTlvTask);
        /* Delete the CLI task */
        if (gCliTask != NULL)
        {
            vTaskDelete(gCliTask);
            gCliTask = NULL;
        }
        /* Get the Frame Periodicity*/

        l_cfgframePeriodus = (gMmwMssMCB.sigProcChainCommonCfg.framePeriodicityus);

        /* Capture the Demo active time in Low Power mode */
        l_demoEndTimeUs = ClockP_getTimeUsec();
        gMmwMssMCB.stats.totalActiveTime_us = (l_demoEndTimeUs - gMmwMssMCB.stats.frameStartTimeStampUs);

        if(gMmwMssMCB.adcDataSourceCfg.source == 1)
        {
            /* In offline ADC injection mode, the total active time (totalActiveTimeUs) is irrelevant; therefore, it is being set to 0 */
            gMmwMssMCB.stats.totalActiveTime_us = 0;
        }

        if (l_cfgframePeriodus > gMmwMssMCB.stats.totalActiveTime_us)
        {
            /* Idle time remaining for Low Power State */
            gMmwMssMCB.stats.ll_FrameIdleTime_us = (unsigned long long)(l_cfgframePeriodus - gMmwMssMCB.stats.totalActiveTime_us);
            /* Low Power mode latency Start time */
            ll_LPmode_LatencyStart = PRCMSlowClkCtrGet();
            /* Radar Power Management Framework driver call for getting to Low Power State */
            Power_idleFunc(gMmwMssMCB.stats.ll_FrameIdleTime_us);
            /* Radar Power Management Framework exits here and control is given back to demo */
            Power_disablePolicy();

            while (gPauseAfterPowerUp)
            {
                ;
            }
            /* Based on idle time left, different low power modes like LPDS, Idle can be taken */
            gDemoLowPwrStateTaken = Power_getLowPowModeTaken();
            if(gDemoLowPwrStateTaken == POWER_NONE)
            {
                /* Use Low Power config only when there is sufficient time. */
                CLI_write("Error: No Sufficient Time for getting into Low Power Modes.\n"); 
                DebugP_assert(0);
            }
        }
        else
        {
            /* Use Low Power config only when there is sufficient time. */
            CLI_write("Error: No Sufficient Time for getting into Low Power Modes.\n"); 
            DebugP_assert(0);
        }
        
        /* Give the gMmwInit semaphore, allowing to start the call to mmwDemo_mmWaveInit. This will parallelize the MMWInit and DPC init */
        xSemaphoreGive(gMmwInit);

        /*If finite frames are configured, stop the demo after configured frames are trasnmitted */
        if((gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames != 0) && \
                (gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames == gMmwMssMCB.stats.frameStartIntCounter))
        {
            rangeProcHWAObj* temp = gMmwMssMCB.rangeProcDpuHandle;
            temp->inProgress = false;
            gMmwMssMCB.oneTimeConfigDone = 0;
            gMmwMssMCB.stats.frameStartIntCounter = 0;
            gSensorStop = 0;
            gIsSensorStarted = 0;
            /* Restart the CLI task */
            CLI_init(CLI_TASK_PRIORITY);
        }
        else
        {
            /* Continue Next frame */
            //MmwStart();
            uint32_t wakeUpSrcMask;
            wakeUpSrcMask = SOC_getWakeupSource();
            gWakeUpSrcMask = wakeUpSrcMask;

            /*Clear the Radar Wakeup Status Register*/
            PRCMClearWakeUpStatus();


            if ((wakeUpSrcMask & SOC_RADAR_WAKEUP_SOURCE_SLEEPCNTR) == SOC_RADAR_WAKEUP_SOURCE_SLEEPCNTR)
            {
                MmwDemo_startDemoProcessing();
            }
            else if ((wakeUpSrcMask & SOC_RADAR_WAKEUP_SOURCE_UART) == SOC_RADAR_WAKEUP_SOURCE_UART)
            {
                rangeProcHWAObj* temp = gMmwMssMCB.rangeProcDpuHandle;
                temp->inProgress = false;
                gMmwMssMCB.oneTimeConfigDone = 0;
                gMmwMssMCB.stats.frameStartIntCounter = 0;
                gSensorStop = 0;
                gIsSensorStarted = 0;

                gMmwMssMCB.mmWaveCfg.initCfg.iswarmstart = FALSE;
                // gMmwMssMCB.sbrCliCurrentCuboidInd = 0;
                // gMmwMssMCB.sbrCliCurrentZoneInd = 0;
                // memset(&gMmwMssMCB.featureExtrModuleCfg.sceneryParams, 0, sizeof(FEXTRACT_sceneryParams));

                gMmwMssMCB.msgIpcCtrlObj.isMsgIpcInitialized = false;

                while (gPauseAfterUartWakeUp)
                {
                    ;
                }

                /* IPC Notify */
                {
                  IpcNotify_Params notifyParams;
                  int32_t status;

                  /* initialize parameters to default */
                  IpcNotify_Params_init(&notifyParams);

                  /* specify the priority of IPC Notify interrupt */
                  notifyParams.intrPriority = 15U;

                  /* specify the core on which this API is called */
                  notifyParams.selfCoreId = CSL_CORE_ID_R5FSS0_0;

                  /* list the cores that will do IPC Notify with this core
                  * Make sure to NOT list 'self' core in the list below
                  */
                  notifyParams.numCores = 1;
                  notifyParams.coreIdList[0] = CSL_CORE_ID_C66SS0;

                  /* initialize the IPC Notify module */
                  status = IpcNotify_init(&notifyParams);
                  DebugP_assert(status==SystemP_SUCCESS);
                }

                /* Power on and unhalt DSP */
                {
                    //volatile CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;

                     /* Restore DSP Clock */
                    ptrDssRcmRegs->DSS_DSP_CLK_SRC_SEL = 0x333;//MMWDEMO_DSP_CLK_SRC_DEFAULT;

                    if((ptrDssRcmRegs->DSP_PD_STATUS & 0x30) == 0x00)
                    {

                        /* Power on DSP */
                        ptrDssRcmRegs->DSS_DSP_RST_CTRL = 0; //Remove from reset state

                        /* Unmask Wakeup Event */
                        ptrDssRcmRegs->DSP_PD_WAKEUP_MASK0 &= 0xFFFEFFFFU;

                        /* Trigger Wakeup */
                        ptrDssRcmRegs->DSP_PD_TRIGGER_WAKUP |= 0x1U;

                        ptrDssRcmRegs->DSS_DSP_L2_PD2_CTRL = 0 ;

                        ptrDssRcmRegs->DSS_DSP_L2_PD4_CTRL = 0 ;

                        /* Trigger Wakeup - Enable Interrupts (unhalt DSP) */
                        ptrDssRcmRegs->DSP_PD_CTRL = 0;

                        ptrDssRcmRegs->DSS_RTIA_CLK_CTRL = 0x0;
                    } else {
                        DebugP_assert(0);
                    }
                }

                /* Restart the CLI task */
                CLI_init(CLI_TASK_PRIORITY);
            }

            if(gMmwMssMCB.adcDataSourceCfg.source == 1)
            {
                /* In test mode trigger next frame processing */
                SemaphoreP_post(&gMmwMssMCB.adcFileTaskSemHandle);
            }
        }
    }
}

void power_idle3entryhook()
{
    uint32_t regVal;

    /* Set R5 Clock Source to OSC */
    SOC_rcmSetR5Clock(40000000,40000000, SOC_RcmR5ClockSource_OSC_CLK);
    do
    {
        regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKSTAT);
    }
    while(((regVal>>4) & 0xFF) != 0x1);
    /* Configure DSP Clock source to OSC */
    SOC_rcmSetDSPClock(40000000,40000000, SOC_RcmDspClockSource_OSC_CLK);
    /* Configure DSP Clock source to OSC */
    SOC_rcmSetDSSClock(40000000,40000000, SOC_RcmDssClockSource_OSC_CLK);

    mmwDemo_freeDmaChannels(gEdmaHandle[0]);
    Drivers_edmaClose();
    EDMA_deinit();
    ADCBuf_close(gMmwMssMCB.adcBuffHandle);
    HWA_close(gHwaHandle);

    /*Changing TOPSS Clock to OSC*/
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_SRCSEL,0x0);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_DIVR,0x0);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL, regVal);

    /* Gating Peripherals */
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_0_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_GATE,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_0_CLKCTL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_GATE,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A0,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0, regVal);

    regVal = HW_RD_REG32(CSL_DSS_RCM_U_BASE + CSL_DSS_RCM_DSS_ADCBUF_CLK_CTRL);
    HW_SET_FIELD32(regVal, CSL_DSS_RCM_DSS_ADCBUF_CLK_CTRL_DSS_ADCBUF_CLK_CTRL_ASSERT,0x7);
    HW_WR_REG32(CSL_DSS_RCM_U_BASE + CSL_DSS_RCM_DSS_ADCBUF_CLK_CTRL, regVal);

    regVal = HW_RD_REG32(CSL_DSS_RCM_U_BASE + CSL_DSS_RCM_DSS_HWA_CLK_CTRL);
    HW_SET_FIELD32(regVal, CSL_DSS_RCM_DSS_HWA_CLK_CTRL_DSS_HWA_CLK_CTRL_ASSERT,0x7);
    HW_WR_REG32(CSL_DSS_RCM_U_BASE + CSL_DSS_RCM_DSS_HWA_CLK_CTRL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_QSPI_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_GATE,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_QSPI_CLKCTL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_I2C_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_GATE,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_I2C_CLKCTL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A,0x7);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0, regVal);

    /*LVDS*/
    regVal = HW_RD_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL0);
    HW_SET_FIELD32(regVal, CSL_TOP_CTRL_LVDS_PAD_CTRL0_LVDS_PAD_CTRL0_CTRL,0x39393939);
    HW_WR_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL0, regVal);

    regVal = HW_RD_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL1);
    HW_SET_FIELD32(regVal, CSL_TOP_CTRL_LVDS_PAD_CTRL1_LVDS_PAD_CTRL1_CTLR,0x01003939);
    HW_WR_REG32(CSL_TOP_CTRL_U_BASE + CSL_TOP_CTRL_LVDS_PAD_CTRL1, regVal);

    /* park pins, based upon board file definitions */
    if (Power_config.pinParkDefs != NULL)
    {
        Power_parkPins(POWER_LPDS);
    }

#if 0
    /* Retention of all memory clusters */
    PRCMSetSRAMRetention(PRCM_APP_PD_SRAM_CLUSTER_1|PRCM_APP_PD_SRAM_CLUSTER_10|PRCM_APP_PD_SRAM_CLUSTER_2| \
                         PRCM_APP_PD_SRAM_CLUSTER_3|PRCM_APP_PD_SRAM_CLUSTER_4|PRCM_APP_PD_SRAM_CLUSTER_5|  \
                         PRCM_APP_PD_SRAM_CLUSTER_6|PRCM_APP_PD_SRAM_CLUSTER_7|PRCM_APP_PD_SRAM_CLUSTER_8|  \
                         PRCM_APP_PD_SRAM_CLUSTER_9|PRCM_DSS_PD_SRAM_CLUSTER_1|PRCM_DSS_PD_SRAM_CLUSTER_2|  \
                         PRCM_DSS_PD_SRAM_CLUSTER_3|PRCM_DSS_PD_SRAM_CLUSTER_4|PRCM_DSS_PD_SRAM_CLUSTER_5|  \
                         PRCM_DSS_PD_SRAM_CLUSTER_6|PRCM_DSS_PD_SRAM_CLUSTER_7|PRCM_FEC_PD_SRAM_CLUSTER_1|  \
                         PRCM_FEC_PD_SRAM_CLUSTER_2|PRCM_FEC_PD_SRAM_CLUSTER_3, PRCM_SRAM_LPDS_RET);
#endif

    /*Disable ADPLL*/
    regVal = HW_RD_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL);
    HW_SET_FIELD32(regVal, CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL_PLL_CLKCTRL_ENSSC,0x0);
    HW_WR_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL, regVal);

    regVal = HW_RD_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL);
    while((regVal&0x4)!=0)
    {
        ;
    }

    regVal = HW_RD_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL);
    HW_SET_FIELD32(regVal, CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL_PLL_CLKCTRL_CLKDCOLDOEN,0x0);
    HW_WR_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL, regVal);

    regVal = HW_RD_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL);
    HW_SET_FIELD32(regVal, CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL_PLL_CLKCTRL_CLKDCOLDOPWDNZ,0x0);
    HW_WR_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL, regVal);

    regVal = HW_RD_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL);
    HW_SET_FIELD32(regVal, CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL_PLL_CLKCTRL_IDLE,0x0);
    HW_WR_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL, regVal);

    regVal = HW_RD_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL);
    HW_SET_FIELD32(regVal, CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL_PLL_CLKCTRL_TINTZ,0x0);
    HW_WR_REG32(CSL_ADPLL_HSDIV_CTRL_U_BASE + CSL_ADPLL_HSDIV_CTRL_PLL_CLKCTRL, regVal);

    /*Disable PLLDIG*/
    regVal = HW_RD_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN);
    HW_SET_FIELD32(regVal, CSL_PLLDIG_CTRL_PLLDIG_EN_PLLDIG_EN_CFG_PLLDIG_EN,0x0);
    HW_WR_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN, regVal);

    /* Powering down DSS PD */
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_DSS_PWR_REQ_PARAM);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_DSS_PWR_REQ_PARAM_DSS_PWR_REQ_PARAM_WAKEUP_OUT_STATE,0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_DSS_PWR_REQ_PARAM, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_DSS_PWR_REQ_PARAM);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_DSS_PWR_REQ_PARAM_DSS_PWR_REQ_PARAM_MODE,0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_DSS_PWR_REQ_PARAM, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DSS_PD_EN);

    while((regVal & 0x100)== 0)
    {
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DSS_PD_EN);
    }

    /* Clock Gating HSM */
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HSM_CLOCK_GATE);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_HSM_CLOCK_GATE_HSM_CLOCK_GATE_CLOCK_GATE,0x7);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HSM_CLOCK_GATE, regVal);

    /* Powering down TEST DEBUG PD */
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_SEL_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

}

void power_idle3resumehook()
{
    volatile uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_DSS_PWR_REQ_PARAM);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_DSS_PWR_REQ_PARAM_DSS_PWR_REQ_PARAM_WAKEUP_OUT_STATE,0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_DSS_PWR_REQ_PARAM, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DSS_PD_EN);
    while((regVal & 0x100)== 1)
    {
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DSS_PD_EN);
    }

    /* Ungate HSM */
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HSM_CLOCK_GATE);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_HSM_CLOCK_GATE_HSM_CLOCK_GATE_CLOCK_GATE,0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HSM_CLOCK_GATE, regVal);  

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);  
    
    PowerClock_init();
    Pinmux_init();

    /*Changing TOPSS Clock back to Fast Clk1 with div by 2*/
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_SRCSEL,0x333);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL, regVal);

    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_TOPSS_CLKCTL_TOPSS_CLKCTL_DIVR,0x111);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_TOPSS_CLKCTL, regVal);

   Drivers_rtiOpen();
   EDMA_init();
   HWA_init();
   ADCBuf_init(SystemP_WAIT_FOREVER);

   SOC_memoryInit(SOC_MEMINIT_APPSS_SHARED_TCMA_BANK0_INIT|SOC_MEMINIT_APPSS_SHARED_TCMA_BANK1_INIT|SOC_MEMINIT_APPSS_SHARED_TCMB_INIT|SOC_MEMINIT_FECSS_SHARED_RAM_INIT|SOC_MEMINIT_DSS_L3_NATIVE_RAM0_INIT|SOC_MEMINIT_DSS_L3_NATIVE_RAM1_INIT|SOC_MEMINIT_APPSS_TPCC_INIT|SOC_MEMINIT_DSS_TPCC_INIT);

   /* DPC initialization */
   DPC_Init();
   Drivers_edmaOpen();

}
