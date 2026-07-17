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

/* mmwave SDK files */
#include <utils/mathutils/mathutils.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <source/mmwave_demo_dss.h>
#include <source/dpc/objectdetection_dss.h>



#define MAXNUMHEAPS (3)
#define L2HEAPSIZE (0x26000)
#define L2SCRATCHSIZE (0x1000) 
#define L1SCRATCHSIZE (0x2000)
#define L1HEAPSIZE (0x2000)
#define DDRHEAPSIZE (0xA0000 - 0x70000)

#pragma DATA_SECTION(ddrHeapMem, ".ddrHeap")
uint8_t ddrHeapMem[DDRHEAPSIZE];

#pragma DATA_SECTION(memHeapL2, ".L2heap")
uint8_t memHeapL2[L2HEAPSIZE];
#pragma DATA_SECTION(l2ScratchMem, ".L2ScratchSect")
uint8_t l2ScratchMem[L2SCRATCHSIZE];

#pragma DATA_SECTION(l1HeapMem, ".L1heap")
uint8_t l1HeapMem[L1HEAPSIZE];
#pragma DATA_SECTION(l1ScratchMem, ".L1ScratchSect")
uint8_t l1ScratchMem[L1SCRATCHSIZE];

/**************************************************************************
 *************************** Macros Definitions ***************************
 **************************************************************************/

/* Enable Continuous wave mode */
#define CONTINUOUS_WAVE_MODE_ENABLE   0

/* Frame ref timer clock for stats info */
#define FRAME_REF_TIMER_CLOCK_MHZ  40


/*! L3 RAM buffer size for object detection DPC */
#define L3_MEM_SIZE (0xB0000 - 0x400) //TODO: Update

/*! Local RAM buffer size for object detection DPC */
#define MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE ((8U+6U+4U+2U+8U) * 1024U) //TODO: Update


// Low power mode defines
#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)


/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
/*! MSS Demo Master Configurations Structure */
MmwDemo_DSS_MCB                 gMmwDssMCB = {0};

/*! L3 RAM buffer for object detection DPC */
//uint8_t                         gMmwL3[L3_MEM_SIZE]  __attribute((section(".bss.l3")));

/*! Local RAM buffer for object detection DPC */
//uint8_t                         gMmwCoreLocMem[MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE];


/*! Task specific declarations */
TaskHandle_t                    gDpcTask;
StaticTask_t                    gDpcTaskObj;
StackType_t                     gDpcTaskStack[DPC_TASK_STACK_SIZE] __attribute__((aligned(32)));



/* For Sensor Stop */
uint32_t                        gSensorStop = 0;

double                          gDemoTimeus, gFrmPrdus;

volatile unsigned long long     gSlpTimeus, gLpdsLatency;

float                           gSocClk = 40000000; //Hz


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
extern int8_t                       gIsSensorStarted;
extern TaskHandle_t                 gDpcTask;

//extern void MmwDemo_dpcTask_dss();

/* Action function  */
typedef void (*ActionFunction)(void * arg);

/* Transition structure */
typedef struct MmwDemo_dss_Transition_t
{
    ActionFunction action;
    MmwDemo_dss_State nextState;
} MmwDemo_dss_Transition;

typedef struct  MmwDemo_Msg_t
{
    uint32_t event;
    uint32_t arg;
} MmwDemo_Msg;


 /* Copy/parse DSP Configuration from API interface structure to preStartCfg */
void mmwave_parseInputConfiguration(DPC_DSS_ObjectDetection_PreStartCfg *out, DPIF_MSS_DSS_PreStartCfg *in)
{
    memset(out, 0, sizeof(DPC_DSS_ObjectDetection_PreStartCfg));

    memcpy(&out->dynCfg.caponChainCfg.dynamicCfarConfig, &in->dynamicCfarConfig, sizeof(RADARDEMO_detectionCFAR_config));

    memcpy(&out->dynCfg.caponChainCfg.doaConfig.rangeAngleCfg, &in->rangeAngleCfg, sizeof(RADARDEMO_aoaEst2D_rangeAngleCfg));
    memcpy(&out->dynCfg.caponChainCfg.doaConfig.angle2DEst.azimElevZoominCfg, &in->angle2DEst.azimElevZoominCfg, sizeof(RADARDEMO_aoaEst2D_2DZoomInCfg));
    memcpy(&out->dynCfg.caponChainCfg.doaConfig.dopCfarCfg, &in->dopCfarCfg, sizeof(RADARDEMO_aoaEst2D_dopCfarCfg));
    memcpy(&out->dynCfg.caponChainCfg.doaConfig.doppBiningCfg, &in->doppBiningCfg, sizeof(RADARDEMO_doppBinSel_config));

    memcpy(&out->dynCfg.caponChainCfg.doaConfig.phaseCompVect, in->phaseCompVect, sizeof(out->dynCfg.caponChainCfg.doaConfig.phaseCompVect));
    memcpy(&out->dynCfg.caponChainCfg.doaConfig.m_ind, &in->m_ind, sizeof(out->dynCfg.caponChainCfg.doaConfig.m_ind));
    memcpy(&out->dynCfg.caponChainCfg.doaConfig.n_ind, &in->n_ind, sizeof(out->dynCfg.caponChainCfg.doaConfig.n_ind));
    memcpy(&out->dynCfg.caponChainCfg.doaConfig.fovCfg, &in->fovCfg, sizeof(out->dynCfg.caponChainCfg.doaConfig.fovCfg));

    out->dynCfg.caponChainCfg.numRangeBins    = in->numRangeBins;
    out->dynCfg.caponChainCfg.rangeFftSize    = in->rangeFftSize;
    out->dynCfg.caponChainCfg.numTxAntenna    = in->numTxAntenna;
    out->dynCfg.caponChainCfg.numPhyRxAntenna = in->numPhyRxAntenna;
    out->dynCfg.caponChainCfg.numAntenna      = in->numTxAntenna * in->numPhyRxAntenna;
    if (out->dynCfg.caponChainCfg.numTxAntenna > 1)
        out->dynCfg.caponChainCfg.mimoModeFlag = 1;
    else
        out->dynCfg.caponChainCfg.mimoModeFlag = 0;
    out->dynCfg.caponChainCfg.numAdcSamplePerChirp       = in->numAdcSamplePerChirp;
    out->dynCfg.caponChainCfg.numChirpPerFrame  = in->numChirpPerFrame;// * gMmwMssMCB.sigProcChainCommonCfg.numFrmPerSlidingWindow;
    out->dynCfg.caponChainCfg.framePeriod       = in->framePeriod;
    out->dynCfg.caponChainCfg.chirpInterval     = in->chirpInterval;
    out->dynCfg.caponChainCfg.bandwidth         = in->bandwidth;
    out->dynCfg.caponChainCfg.centerFreq        = in->centerFreq;
    out->dynCfg.caponChainCfg.maxNumDetObj      = in->maxNumDetObj;
    out->dynCfg.caponChainCfg.numFrmPerSlidingWindow = in->numFrmPerSlidingWindow;
    out->dynCfg.caponChainCfg.dynamicSideLobeThr = in->dynamicSideLobeThr;
    out->dynCfg.caponChainCfg.staticSideLobeThr = in->staticSideLobeThr;


    out->shareMemCfg.radarCubeMem.addr = in->radarCube.data;
    out->shareMemCfg.radarCubeMem.size = in->radarCube.dataSize;
    out->shareMemCfg.shareMemEnable = true;
    out->dynCfg.radarCubeFormat = in->radarCube.datafmt;

    out->dynCfg.caponChainCfg.exportCoarseHeatmap = in->exportCoarseHeatmap;
    out->dynCfg.caponChainCfg.exportRawCfarDetList = in->exportRawCfarDetList;
    out->dynCfg.caponChainCfg.exportZoomInHeatmap = in->exportZoomInHeatmap;
}


/* Action functions for each transition */
void action_DPC_config(void * arg)
{
    int32_t errorCode = 0;
    DPIF_MSS_DSS_PreStartCfg *dpifPreStartCfg = (DPIF_MSS_DSS_PreStartCfg *) arg;
    DPC_DSS_ObjectDetection_PreStartCfg preStartCfg;


    /* Copy/parse DSP Configuration from API interface structure to preStartCfg */
    mmwave_parseInputConfiguration(&preStartCfg, dpifPreStartCfg);

    gMmwDssMCB.numFrmPerSlidingWindow = dpifPreStartCfg->numFrmPerSlidingWindow;

    errorCode = DPC_ObjDetDSP_preStartConfig(&preStartCfg);
    if (errorCode != 0)
    {
        DebugP_assert(0);
    }

    MsgIpc_sendMessage(&gMmwDssMCB.msgIpcCtrlObj, DPC_DSS_TO_MSS_CONFIGURATION_COMPLETED, NULL);
    gMmwDssMCB.interSubFrameProcToken--;
    gMmwDssMCB.dssConfigurationCntr++;

    /* For debugging only */
    gMmwDssMCB.disablePointCloudGeneration = dpifPreStartCfg->disablePointCloudGeneration;
}
void action_DPC_exec(void *arg)
{

    DPC_ObjectDetection_execute();

    MsgIpc_sendMessage(&gMmwDssMCB.msgIpcCtrlObj, DPC_DSS_TO_MSS_POINT_CLOUD_READY, (uint32_t) gMmwDssMCB.outputFromDSP);
    gMmwDssMCB.interSubFrameProcToken--;
    gMmwDssMCB.radarCubeReadyEventCntr++;

    gMmwDssMCB.frmCntrModNumFrmPerSlidWin++;
    if (gMmwDssMCB.frmCntrModNumFrmPerSlidWin >=  gMmwDssMCB.numFrmPerSlidingWindow)
    {
        gMmwDssMCB.frmCntrModNumFrmPerSlidWin = 0;
    }
}
void action_DPC_stop(void *arg)
{
    printf("Action: Stop.\n");
}
void action_DPC_noop(void *arg)
{
    printf("Action: No operation.\n");
}

/* DDS DPC FSM Transition Table */
MmwDemo_dss_Transition MmwDemo_FSM_Table[NUM_STATES][DPC_MSS_TO_DSS_NUM_EVENTS] = {
        [STATE_IDLE][DPC_MSS_TO_DSS_PRE_START_CONFIG]    = {action_DPC_config, STATE_READY},
        [STATE_IDLE][DPC_MSS_TO_DSS_RADAR_CUBE_READY]    = {action_DPC_noop, STATE_IDLE},
        [STATE_IDLE][DPC_MSS_TO_DSS_STOP]                = {action_DPC_noop, STATE_IDLE},

        [STATE_READY][DPC_MSS_TO_DSS_PRE_START_CONFIG]   = {action_DPC_config, STATE_READY},
        [STATE_READY][DPC_MSS_TO_DSS_RADAR_CUBE_READY]   = {action_DPC_exec, STATE_READY},
        [STATE_READY][DPC_MSS_TO_DSS_STOP]               = {action_DPC_noop, STATE_READY},
};


/* Function to handle an event by looking it up in the FSM table */
void MmwDemo_handleMsg(MmwDemo_Msg *msg) {
    MmwDemo_dss_Transition transition;

    if (msg->event >=  DPC_MSS_TO_DSS_NUM_EVENTS)
    {
        DebugP_assert(0);
    }
    transition = MmwDemo_FSM_Table[gMmwDssMCB.currentState][msg->event];

    /* Execute the action function */
    if (transition.action) {
        transition.action((void *) msg->arg);
    }

    /* Update the state */
    gMmwDssMCB.currentState = transition.nextState;
}


/* Registered function with IPC driver that receives messages from MSS */
void DPC_dss_MsgHandler(uint32_t remoteCoreId, uint16_t localClientId, uint64_t msgValue, int32_t crcStatus, void *arg)
{
    uint32_t message;
    MmwDemo_Msg msg;
    msg.event = (uint32_t) ((msgValue >> 32) & 0xffff);
    msg.arg = (uint32_t) (0x00000000FFFFFFFF & msgValue);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(gMmwDssMCB.eventQueue != NULL)
    {
        /* Check for previous completion */
        if (gMmwDssMCB.interSubFrameProcToken != 0)
        {
            gMmwDssMCB.interSubFrameProcOverflowCntr++;
        }
        gMmwDssMCB.interSubFrameProcToken++;

        /* Send the event to the queue from ISR context */
        if (xQueueSendFromISR(gMmwDssMCB.eventQueue, &msg, &xHigherPriorityTaskWoken) != pdPASS) {
            DebugP_assert(0);
        }
    }
    else
    {
        DebugP_assert(0);
    }
    // Context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

volatile uint32_t gDummy = 0;

//#define DEBUG_TEST_PROFILE_FAST_LOG2_FUNCTION
#ifdef DEBUG_TEST_PROFILE_FAST_LOG2_FUNCTION
extern float fast_log2(float x);
//DEBUG TESTING REMOVE THIS
volatile float yy[100];
volatile float yyVal;
#endif

void people_tracking_6844_dss(void* args)
{
    int32_t errorCode = SystemP_SUCCESS;
    int32_t retVal = -1;
    MmwDemo_Msg msg;
    MsgIpc_Cfg msgIpcCfg;
    DPC_DSS_ObjectDetection_InitParams objDetInitParams;

    //CacheP_disable(CacheP_TYPE_L1D);
    //CacheP_disable(CacheP_TYPE_L2D);

    //DEBUG TESTING LOG2 function
    TSCL = 0;


    /* Peripheral Driver Initialization */
    Drivers_open();
    Board_driversOpen();

#ifdef DEBUG_TEST_PROFILE_FAST_LOG2_FUNCTION
    {
        //DEBUG TESTING LOG2 function
        uint32_t t1 = TSCL;
        yyVal = 1.;
        for (int ii=0; ii<100; ii++)
        {
            yy[ii] = fast_log2(yyVal);
            yyVal = yyVal + 1.;
        }
        uint32_t tProc = TSCL - t1;
        printf("Log2 fast = %d\n cycles", tProc);

        //DEBUG TESTING LOG2 function
        t1 = TSCL;
        yyVal = 1.;
        for (int ii=0; ii<100; ii++)
        {
            yy[ii] = (float) log2(yyVal);
            yyVal = yyVal + 1.;
        }
        tProc = TSCL - t1;
        printf("Log2 math = %d\n cycles", tProc);

    }
#endif

    /*Init DPC*/
    memset((void *)&objDetInitParams, 0, sizeof(DPC_DSS_ObjectDetection_InitParams));
    /*Set up init params for memory osal*/
    objDetInitParams.L3HeapCfg.addr    = (void *)&ddrHeapMem[0];
    objDetInitParams.L3HeapCfg.size    = DDRHEAPSIZE;
    objDetInitParams.L3ScratchCfg.addr = (void *)NULL;
    objDetInitParams.L3ScratchCfg.size = 0;

    objDetInitParams.CoreL2HeapCfg.addr    = (void *)&memHeapL2[0];
    objDetInitParams.CoreL2HeapCfg.size    = L2HEAPSIZE;
    objDetInitParams.CoreL2ScratchCfg.addr = (void *)&l2ScratchMem[0];
    objDetInitParams.CoreL2ScratchCfg.size = L2SCRATCHSIZE;

    objDetInitParams.CoreL1HeapCfg.addr    = (void *)&l1HeapMem[0];
    objDetInitParams.CoreL1HeapCfg.size    = L1HEAPSIZE;
    objDetInitParams.CoreL1ScratchCfg.addr = (void *)&l1ScratchMem[0];
    objDetInitParams.CoreL1ScratchCfg.size = L1SCRATCHSIZE;
    DPC_ObjectDetection_init(&objDetInitParams, &errorCode);

    /* Configure IPC Messaging */
    msgIpcCfg.msgChanId = 1;
    msgIpcCfg.remoteCoreId = CSL_CORE_ID_R5FSS0_0;
    msgIpcCfg.msgCallback = (IpcNotify_FxnCallback)DPC_dss_MsgHandler;
    msgIpcCfg.arg = NULL;

    MsgIpc_Config(&gMmwDssMCB.msgIpcCtrlObj, &msgIpcCfg);

    /* Create the event queue that receives events from R5F */
    gMmwDssMCB.eventQueue = xQueueCreate(8, sizeof(MmwDemo_Msg));
    if (gMmwDssMCB.eventQueue == NULL)
    {
        DebugP_assert(0);
    }

    /* Set FSM Current state */
    gMmwDssMCB.currentState = STATE_IDLE;
    /* Sync with MSS */
    MsgIpc_Sync();


     /* Handles events from the queue and processes the FSM  */
     while (1)
     {
        // Wait for an event from the queue
        if (xQueueReceive(gMmwDssMCB.eventQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            MmwDemo_handleMsg(&msg);
        }

        if (gDummy)
        {
            break;
        }
     }


    /* Create binary semaphores to pend at different stages of the OOB */
    errorCode = SemaphoreP_constructBinary(&gMmwDssMCB.demoInitTaskCompleteSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    /* Never return for this task. */
    SemaphoreP_pend(&gMmwDssMCB.demoInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);

    Board_driversClose();
    Drivers_close();
}
