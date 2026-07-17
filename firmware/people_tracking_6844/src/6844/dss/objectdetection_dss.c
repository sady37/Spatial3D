/*
 *   @file  objectdetection.c
 *
 *   @brief
 *      Object Detection DPC implementation using DSP.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
/* Standard Include Files. */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>

/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>

#include <drivers/ipc_notify.h>

/* mmwave SDK files */
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <utils/mathutils/mathutils.h>
#include <source/mmwave_demo_dss.h>


/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
//#pragma diag_push
//#pragma diag_suppress 48
//#include <ti/mathlib/mathlib.h>
//#pragma diag_pop

/*! This is supplied at command line when application builds this file. This file
 * is owned by the application and contains all resource partitioning, an
 * application may include more than one DPC and also use resources outside of DPCs.
 * The resource definitions used by this object detection DPC are prefixed by DPC_OBJDET_ */


/* Obj Det instance etc */
#include <source/dpc/objectdetection_dss_internal.h>
#include <source/dpc/objectdetection_dss.h>
#include <common_mss_dss/msg_ipc/msg_ipc.h>

/**************************************************************************
 ************************** External Definitions **************************
 **************************************************************************/
extern MmwDemo_DSS_MCB gMmwDssMCB;

/**************************************************************************
 ************************** Local Definitions *****************************
 **************************************************************************/

/**
@}
*/
/*! Maximum Number of objects that can be detected in a frame */
#define DPC_OBJDET_MAX_NUM_OBJECTS DOA_OUTPUT_MAXPOINTS

/**************************************************************************
 ************************** Local Functions Prototype **************************
 **************************************************************************/


#if 0
static int32_t DPC_ObjectDetection_execute(
    DPM_DPCHandle handle,
    DPM_Buffer   *ptrResult);

static int32_t DPC_ObjectDetection_ioctl(
    DPM_DPCHandle handle,
    uint32_t      cmd,
    void         *arg,
    uint32_t      argLen);

static int32_t DPC_ObjectDetection_start(DPM_DPCHandle handle);
static int32_t DPC_ObjectDetection_stop(DPM_DPCHandle handle);
static int32_t DPC_ObjectDetection_deinit(DPM_DPCHandle handle);
static void    DPC_ObjectDetection_frameStart(DPM_DPCHandle handle);
int32_t        DPC_ObjectDetection_dataInjection(DPM_DPCHandle handle, DPM_Buffer *ptrBuffer);
#endif

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/


/**
 *  @b Description
 *  @n
 *     Performs processing related to pre-start configuration
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
int32_t DPC_ObjDetDSP_preStartConfig(DPC_DSS_ObjectDetection_PreStartCfg *preStartCfg)
{
    int32_t                     retVal = 0;
    DPC_DSS_ObjectDetection_DynCfg *dynCfg = &gMmwDssMCB.objDetObj.dynCfg;
    DPIF_RadarCube              radarCube;
    DPU_ProcessErrorCodes       procErrorCode = PROCESS_OK;

    /*Copy configuration message received from MSS to local memory  */
    memcpy(&gMmwDssMCB.objDetObj.dynCfg, &preStartCfg->dynCfg, sizeof(DPC_DSS_ObjectDetection_DynCfg));

    //radarOsal_memResetHeapAll();

    /* Check radar cube size */
    radarCube.dataSize = dynCfg->caponChainCfg.numRangeBins *
                         dynCfg->caponChainCfg.numChirpPerFrame *  //Includes number of frames per slidingWindw
                         dynCfg->caponChainCfg.numTxAntenna *
                         dynCfg->caponChainCfg.numPhyRxAntenna * sizeof(cplx16_t);
    if (preStartCfg->shareMemCfg.shareMemEnable == true)
    {
        if ((preStartCfg->shareMemCfg.radarCubeMem.addr != NULL) &&
            (preStartCfg->shareMemCfg.radarCubeMem.size == radarCube.dataSize))
        {
            /* Use assigned radar cube address */
            radarCube.data = preStartCfg->shareMemCfg.radarCubeMem.addr;
        }
        else
        {
            retVal = DPC_OBJECTDETECTION_EINVAL__COMMAND;
            goto exit;
        }
#ifdef RADARDEMO_AOARADARCUDE_RNGCHIRPANT
        if (dynCfg->radarCubeFormat != DPIF_RADARCUBE_FORMAT_2)
        {
            retVal = DPC_OBJECTDETECTION_EINVAL_CUBE;
            goto exit;
        }
#endif
    }
    else
    {
        retVal = DPC_OBJECTDETECTION_EINVAL_CUBE;
        goto exit;
    }

    radarCube.datafmt   = DPIF_RADARCUBE_FORMAT_2;
    memcpy(&gMmwDssMCB.objDetObj.radarCube, &radarCube, sizeof(DPIF_RadarCube));

    gMmwDssMCB.objDetObj.dpuCaponObj = DPU_radarProcess_init(&gMmwDssMCB.objDetObj.dynCfg.caponChainCfg, &procErrorCode);
    if (procErrorCode > PROCESS_OK)
    {
        retVal = DPC_OBJECTDETECTION_EINTERNAL;
        //DebugP_log1("DPC config error %d\n", procErrorCode);
        goto exit;
    }

    /* Report RAM usage */
    radarOsal_printHeapStats();

    /* Set output pointer */
    gMmwDssMCB.outputFromDSP = &gMmwDssMCB.objDetObj.executeResult->objOut;
    if(gMmwDssMCB.outputFromDSP == NULL)
    {
        retVal = DPC_OBJECTDETECTION_EINTERNAL;
        goto exit;
    }

exit:
    return retVal;
}

//Used for testing/debugging:
volatile float gRange = 1.1;
volatile float gAzimuth = -27.;
volatile float gElevation = 23.;


/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) execute function which is invoked by the application
 *      in the DPM's execute context when the DPC issues DPM_notifyExecute API from
 *      its registered @ref DPC_ObjectDetection_frameStart API that is invoked every
 *      frame interrupt.
 *
 *  @param[in]  handle       DPM's DPC handle
 *  @param[out]  ptrResult   Pointer to the result
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t DPC_ObjectDetection_execute()
{
    int32_t procErrorCode = PROCESS_OK;
    int32_t             retVal = 0;
    volatile uint32_t   startTime;
    int32_t             i;
    uint32_t            frmCntr = gMmwDssMCB.frmCntrModNumFrmPerSlidWin;

    ObjDetObj *objDetObj = &gMmwDssMCB.objDetObj;
    cplx16_t *pDataIn = (cplx16_t *) objDetObj->radarCube.data;
    DPIF_MSS_DSS_radarProcessOutput *result = &objDetObj->executeResult->objOut;
    DPU_radarProcess_Handle *dpuCaponObj = gMmwDssMCB.objDetObj.dpuCaponObj;


    //startTime = Cycleprofiler_getTimeStamp();
    if (!gMmwDssMCB.disablePointCloudGeneration)
    {
        DPU_radarProcess_process(dpuCaponObj, pDataIn, frmCntr, result, &procErrorCode);
        if (procErrorCode > PROCESS_OK)
        {
            retVal = -1;
            goto exit;
        }
        // writeback all the data shared with R4 in L3, and prepare cache for next frames radar cube from R4.
        CacheP_wbInvAll(CacheP_TYPE_L1D);
    }

    //DebugP_log0("ObjDet DPC: Frame Proc Done\n");

    //objDetObj->stats->interFrameEndTimeStamp = Cycleprofiler_getTimeStamp();
    //memcpy(&(objDetObj->stats->subFrbenchmarkDetails), result->benchmarkOut, sizeof(radarProcessBenchmarkElem));
    //objDetObj->stats->interFrameExecTimeInUsec  = (uint32_t)((float)(objDetObj->stats->interFrameEndTimeStamp - objDetObj->stats->frameStartTimeStamp) * _rcpsp((float)DSP_CLOCK_MHZ));
    //objDetObj->stats->activeFrameProcTimeInUsec = (uint32_t)((float)(objDetObj->stats->interFrameEndTimeStamp - startTime) * _rcpsp((float)DSP_CLOCK_MHZ));

    if (gMmwDssMCB.disablePointCloudGeneration)
    {
        int ii;
        //TEMP Remove this. This is temporary for debugging:
        gMmwDssMCB.outputFromDSP->pointCloudOut.object_count = 0;
        for(ii = 0; ii<gMmwDssMCB.outputFromDSP->pointCloudOut.object_count; ii++)
        {
            gMmwDssMCB.outputFromDSP->pointCloudOut.pointCloud[ii].azimuthAngle = (gAzimuth + 5.*((6.1035e-05 * (float) rand()) - 1.))*3.1415926/180.;
            gMmwDssMCB.outputFromDSP->pointCloudOut.pointCloud[ii].elevAngle = (gElevation + 5.*((6.1035e-05 * (float) rand()) - 1.))*3.1415926/180.;
            gMmwDssMCB.outputFromDSP->pointCloudOut.pointCloud[ii].range = gRange + 0.1*((6.1035e-05 * (float) rand()) - 1.);
            gMmwDssMCB.outputFromDSP->pointCloudOut.pointCloud[ii].velocity = 0.1*((6.1035e-05 * (float) rand()) - 1.);

            gMmwDssMCB.outputFromDSP->pointCloudOut.snr[ii].snr = 10. + 1.*((6.1035e-05 * (float) rand()) - 1.);
        }
        // writeback all the data shared with R4 in L3, and prepare cache for next frames radar cube from R4.
        CacheP_wbInvAll(CacheP_TYPE_L1D);
    }


exit:

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) initialization function which is invoked by the
 *      application using DPM_init API. Among other things, this API allocates DPC instance
 *      and DPU instances (by calling DPU's init APIs) from the MemoryP osal
 *      heap. If this API returns an error of any type, the heap is not guaranteed
 *      to be in the same state as before calling the API (i.e any allocations
 *      from the heap while executing the API are not guaranteed to be deallocated
 *      in case of error), so any error from this API should be considered fatal and
 *      if the error is of _ENOMEM type, the application will
 *      have to be built again with a bigger heap size to address the problem.
 *
 *  @param[in]  dpmHandle   DPM's DPC handle
 *  @param[in]  ptrInitCfg  Handle to the framework semaphore
 *  @param[out] errCode     Error code populated on error
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void DPC_ObjectDetection_init(DPC_DSS_ObjectDetection_InitParams *dpcInitParams,
                                              int32_t *errCode)
{
    ObjDetObj *objDetObj;
    radarOsal_heapConfig heapconfig[3];
    *errCode = 0;

    /*Set up heap and mem osal*/
    {
        memset(heapconfig, 0, sizeof(heapconfig));
        heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType    = RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
        heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr    = (int8_t *)dpcInitParams->L3HeapCfg.addr;
        heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize    = dpcInitParams->L3HeapCfg.size;
        heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr = (int8_t *)dpcInitParams->L3ScratchCfg.addr;
        heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize = dpcInitParams->L3ScratchCfg.size;

        heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType    = RADARMEMOSAL_HEAPTYPE_LL2;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr    = (int8_t *)dpcInitParams->CoreL2HeapCfg.addr;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize    = dpcInitParams->CoreL2HeapCfg.size;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr = (int8_t *)dpcInitParams->CoreL2ScratchCfg.addr;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize = dpcInitParams->CoreL2ScratchCfg.size;

        heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapType    = RADARMEMOSAL_HEAPTYPE_LL1;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr    = (int8_t *)dpcInitParams->CoreL1HeapCfg.addr;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapSize    = dpcInitParams->CoreL1HeapCfg.size;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr = (int8_t *)dpcInitParams->CoreL1ScratchCfg.addr;
        heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize = dpcInitParams->CoreL1ScratchCfg.size;
        if (radarOsal_memInit(&heapconfig[0], 3) == RADARMEMOSAL_FAIL)
        {
            *errCode = DPC_OBJECTDETECTION_MEMINITERR;
            goto exit;
        }
    }
    objDetObj = &gMmwDssMCB.objDetObj;  //MemoryP_ctrlAlloc(sizeof(ObjDetObj), 0);


    /* Initialize memory */
    memset((void *)objDetObj, 0, sizeof(ObjDetObj));


    objDetObj->executeResult = (DPC_DSS_ObjectDetection_ExecuteResult *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(DPC_DSS_ObjectDetection_ExecuteResult), 1);
    objDetObj->stats         = (DPC_DSS_ObjectDetection_Stats *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(DPC_DSS_ObjectDetection_Stats), 1);

    //*errCode = DPC_ObjDetDSP_initDPU(objDetObj, DEMO_RL_MAX_SUBFRAMES);
    // printf ("DPC init done!\n");

exit:
    return;
    //return ((DPM_DPCHandle)objDetObj);
}

#if 0
/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) de-initialization function which is invoked by the
 *      application using DPM_deinit API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_deinit(DPM_DPCHandle handle)
{
    ObjDetObj *objDetObj = (ObjDetObj *)handle;
    int32_t    retVal    = 0;

    if (handle == NULL)
    {
        retVal = DPC_OBJECTDETECTION_EINVAL;
        goto exit;
    }

    retVal = DPC_ObjDetDSP_deinitDPU(objDetObj, DEMO_RL_MAX_SUBFRAMES);

    MemoryP_ctrlFree(handle, sizeof(ObjDetObj));

exit:
    return (retVal);
}
#endif
/**************************************************************************
 ************************* Global Declarations ****************************
 **************************************************************************/

/* @} */
