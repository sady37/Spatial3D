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
/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/CycleCounterP.h>

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

#include <source/mmwave_control/interrupts.h>
#include <source/mmwave_demo_mss.h>
#include <source/utils/mmw_demo_utils.h>
#include <source/mmwave_control/monitors.h>
#include <source/dpc/dpc_mss.h>

#define FRAME_REF_TIMER_CLOCK_MHZ  40
#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)

extern MmwDemo_MSS_MCB gMmwMssMCB;

// LED config
extern uint32_t gGpioBaseAddrLed, gPinNumLed;

HwiP_Object gHwiChirpAvailableHwiObject;
HwiP_Object gHwiFrameStartHwiObject;

/* Local functions */
#ifdef ENABLE_BURST_INTERRUPT
void mmwDemoBurstISR(void *arg);
#endif

#if (_DEBUG_ == 1)
/* In debug build, in order to debug target code (set breakpoints, step over...) below variable is set to 1.
 * It will prevent ISR mmwDemoFrameStartISR from forcing the code to stop */
volatile uint32_t gDebugTargetCode = 1;
#else
volatile uint32_t gDebugTargetCode = 1;
#endif

#if (ENABLE_MONITORS==1)
/**
 *  @b Description
 *  @n
 *      This is the ISR Handler for Monitors
 */
void mmwDemoMonitorISR(void)
{
    /*Clear the interrupt*/
    HwiP_clearInt(CSL_APPSS_INTR_FEC_INTR2);
    mmwDemo_GetMonRes();
    /*Posting the semaphore if LowPowerMode is enabled*/
    if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE) || (gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE))
    {
        SemaphoreP_post(&gMmwMssMCB.rfmonSemHandle);
    }
}
#endif

/* For debugging purposes */
#if 0
volatile uint32_t gMmwDemoChirpStartCnt = 0;
#endif
#ifdef ENABLE_BURST_INTERRUPT
volatile uint32_t gMmwDemoBurstCnt = 0;
#endif

/* For debugging purposes */
#ifdef ENABLE_BURST_INTERRUPT
/**
 *  @b Description
 *  @n
 *      This is to register Burst Interrupt
 */
int32_t MmwDemo_registerBurstInterrupt(void)
{
    int32_t           regVal, retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;

    // Configure the interrupt for Burst End
    regVal = HW_RD_REG32(CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_IRQ_REQ_SEL);
    regVal = regVal | 0x1000;
    HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_IRQ_REQ_SEL), regVal);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSL_APPSS_INTR_FECSS_CHIRPTIMER_AND_BURST_START_AND_BURST_END;
    hwiPrms.callback    = mmwDemoBurstISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_FECSS_CHIRPTIMER_AND_BURST_START_AND_BURST_END);
    }

    return retVal;
}
#endif
#if 0
/**
 *  @b Description
 *  @n
 *      This is to register Chirpt Interrupt
 */
int32_t MmwDemo_registerChirpInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSL_APPSS_INTR_FECSS_CHIRPTIMER_AND_CHIRP_START_AND_CHIRP_END;
    hwiPrms.callback    = mmwDemoChirpStartISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_FECSS_CHIRPTIMER_AND_CHIRP_START_AND_CHIRP_END);
    }

    return retVal;
}
#endif
//#ifdef ENABLE_CHIRP_AVAILABLE_INTERRUPT
/**
 *  @b Description
 *  @n
 *      This is to register Chirp Available Interrupt
 */
int32_t MmwDemo_registerChirpAvailableInterrupts(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSL_APPSS_INTR_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN;
    hwiPrms.callback    = mmwDemoChirpISR;
    /* Use this to change the priority */
    hwiPrms.priority    = 5;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN);
    }

    return retVal;
}
//#endif

/**
 *  @b Description
 *  @n
 *      This is to register Frame Start Interrupt
 */
int32_t MmwDemo_registerFrameStartInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START;
    hwiPrms.callback    = mmwDemoFrameStartISR;
    /* Use this to change the priority */
    //hwiPrms.priority    = 0;
    hwiPrms.args        = (void *) &gMmwMssMCB;
    status              = HwiP_construct(&gHwiFrameStartHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);
    }

    return retVal;
}

#ifdef ENABLE_BURST_INTERRUPT
volatile uint32_t gBurstTime[128];
volatile uint32_t gBurstTimeInd = 0;
#endif

/* For debugging purposes*/
#ifdef ENABLE_BURST_INTERRUPT
/**
*  @b Description
*  @n
*    Burst ISR
*/
void mmwDemoBurstISR(void *arg)
{
    HwiP_clearInt(CSL_APPSS_INTR_FECSS_CHIRPTIMER_AND_BURST_START_AND_BURST_END);
    gMmwDemoBurstCnt++;

    if (gBurstTimeInd < 128)
    {
        gBurstTime[gBurstTimeInd++] = Cycleprofiler_getTimeStamp();//CycleCounterP_getCount32();
    }
}
#endif
#if 0
/**
*  @b Description
*  @n
*    Chirp Start ISR
*/
void mmwDemoChirpStartISR(void *arg)
{
    HwiP_clearInt(CSL_APPSS_INTR_FECSS_CHIRPTIMER_AND_CHIRP_START_AND_CHIRP_END);
}
#endif
//#ifdef ENABLE_CHIRP_AVAILABLE_INTERRUPT
uint32_t gChirpTimeProfileBuf[128];
volatile uint32_t gChirpTimeProfileBufInd = 0;

/**
*  @b Description
*  @n
*    Chirp ISR
*/
static void mmwDemoChirpISR(void *arg)
{
    HwiP_clearInt(CSL_APPSS_INTR_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN);
    /* If ADC logging via LVDS is enabled, trigger the edma transfer from adcbuf to cbuff, so that LVDS stream will start upon start of chirp */
	if (gMmwMssMCB.adcLogging.enable == 1) 
    {
        /* Trigger the edma transfer from adcbuf to cbuff, so that LVDS stream will start upon start of chirp */
        MmwDemo_configLVDSDataTrigger();
    }

    if (gChirpTimeProfileBufInd < 128)
    {
        gChirpTimeProfileBuf[gChirpTimeProfileBufInd++] = Cycleprofiler_getTimeStamp();//CycleCounterP_getCount32();
    }
}
//#endif

/**
*  @b Description
*  @n
*    Frame start ISR
*/
static void mmwDemoFrameStartISR(void *arg)
{
    uint64_t l_demoStartTimeUs;
    unsigned long long ll_startTimeSlowClk;

    uint32_t curCycle;
    MmwDemo_MSS_MCB *mmwMssMCB = (MmwDemo_MSS_MCB *) arg;

    HwiP_clearInt(CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);

    /* Capture the frame start time using FreeRTOS timer */
    curCycle = Cycleprofiler_getTimeStamp();
    l_demoStartTimeUs = ClockP_getTimeUsec();
    /* Capture the frame start time using the Slow Clock. This is needed when Low power mode is enabled */
    ll_startTimeSlowClk = PRCMSlowClkCtrGet();

    if (gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_DISABLE)
    {
        /* For testing */
        mmwMssMCB->stats.framePeriod_us = (curCycle - mmwMssMCB->stats.frameStartTimeStamp[(mmwMssMCB->stats.frameStartIntCounter-1) & 0x3])/FRAME_REF_TIMER_CLOCK_MHZ;
        mmwMssMCB->stats.frameStartTimeStamp[mmwMssMCB->stats.frameStartIntCounter & 0x3] = curCycle;

        GPIO_pinWriteHigh(gGpioBaseAddrLed, gPinNumLed);
    }
    else
    {
        /* FreeRTOS timer is shutdown during Low Power mode. Hence Slow Clock has to be used when Low power mode is enabled */
        mmwMssMCB->stats.framePeriod_us = round((ll_startTimeSlowClk - mmwMssMCB->stats.frameStartTimeStampSlowClk) * M_TICKS_TO_USEC_SLOWCLK);
    }
    mmwMssMCB->stats.frameStartTimeStampUs = l_demoStartTimeUs;

    mmwMssMCB->stats.frameStartTimeStampSlowClk = ll_startTimeSlowClk;


    if (gDebugTargetCode == 0)
    {
        DebugP_assert(mmwMssMCB->interSubFrameProcToken == 0);
    }

    if(mmwMssMCB->interSubFrameProcToken > 0)
    {
        mmwMssMCB->interSubFrameProcOverflowCntr++;
    }

    mmwMssMCB->interSubFrameProcToken++;

    mmwMssMCB->stats.frameStartIntCounter++;

    // if ((mmwMssMCB->runningMode == DPC_RUNNING_MODE_INDET) && (mmwMssMCB->lowPowerMode == LOW_PWR_MODE_ENABLE))
    // {
    //     //Change to oscilator clock (40MHz)
    //     SOC_rcmSetR5Clock(SOC_RCM_XTAL_CLK_40MHZ,SOC_RCM_XTAL_CLK_40MHZ, SOC_RcmR5ClockSource_OSC_CLK);
    // }
}
