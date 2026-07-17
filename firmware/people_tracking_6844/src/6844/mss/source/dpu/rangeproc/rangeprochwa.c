/*
 *  NOTE:
 *      (C) Copyright 2024 Texas Instruments, Inc.
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
/**
 *   @file  rangeprochwa.c
 *
 *   @brief
 *      Implements Range FFT data processing Unit using HWA(Isolated non-interleaved). 
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

// #define SOC_AWR294X

/* MCU+SDK Include files */
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#ifdef SUBSYS_MSS
#include <kernel/dpl/CacheP.h>
#endif

/* Data Path Include files */
#include <datapath/dpu/rangeproc/v1/rangeprochwa.h>

/* MATH utils library Include files */
#include <utils/mathutils/mathutils.h>

/* Internal include Files */
#include <datapath/dpu/rangeproc/v1/rangeprochwa_internal.h>

/* User defined heap memory and handle */
#define RANGEPROCHWA_HEAP_MEM_SIZE  (sizeof(rangeProcHWAObj))

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

#define DPU_RANGEHWA_MEM_BANK_INDX_SRC_PING   0
#define DPU_RANGEHWA_MEM_BANK_INDX_SRC_PONG   1
#define DPU_RANGEHWA_MEM_BANK_INDX_DST_PING   2
#define DPU_RANGEHWA_MEM_BANK_INDX_DST_PONG   3

#define DPU_RANGEHWA_SRCADDR_PING   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PING])
#define DPU_RANGEHWA_SRCADDR_PONG   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PONG])
#define DPU_RANGEHWA_DSTADDR_PING   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_DST_PING])
#define DPU_RANGEHWA_DSTADDR_PONG   HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_DST_PONG])

rangeProcHWAObj RangeObj; //TODO: Heap mem_alloc??

/**************************************************************************
 ************************ Internal Functions Prototype       **********************
 **************************************************************************/

static void rangeProcHWADoneIsrCallback(void * arg);

void rangeProcHWA_EDMA_transferCompletionCallbackFxn(Edma_IntrHandle intrHandle,
   void *args);

static int32_t rangeProcHWA_ConfigHWA
(
    rangeProcHWAObj     *rangeProcObj,
    uint8_t     destChanPing,
    uint8_t     destChanPong,
    uint32_t    hwaMemSrcPingOffset,
    uint32_t    hwaMemSrcPongOffset,
    uint32_t    hwaMemDestPingOffset,
    uint32_t    hwaMemDestPongOffset
);

static int32_t rangeProcHWA_ConfigHWACommon
(
    rangeProcHWAObj     *rangeProcObj
);

static int32_t rangeProcHWA_TriggerHWA
(
    rangeProcHWAObj     *rangeProcObj
);

static int32_t rangeProcHWA_ConfigEDMA_DataOut
(
    rangeProcHWAObj         *rangeProcObj,
    rangeProc_dpParams      *DPParams,
    DPU_RangeProcHWA_HW_Resources *pHwConfig,
    uint32_t                hwaOutPingOffset,
    uint32_t                hwaOutPongOffset
);
static int32_t rangeProcHWA_ConfigEDMA_DataIn
(
    rangeProcHWAObj         *rangeProcObj,
    rangeProc_dpParams      *DPParams,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
);

static int32_t rangeProcHWA_Config
(
    rangeProcHWAObj          *rangeProcObj,
    rangeProc_dpParams       *DPParams,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
);

static int32_t rangeProcHWA_ConfigEDMADummyTwoLinks
(
    EDMA_Handle             handle,
    DPEDMA_4LinkChanCfg     *chanCfg,
    uint8_t                 chainChId0,
    uint8_t                 chainChId1,
    uint16_t                numIter
);

static int32_t rangeProcHWA_ConfigEDMADummyFourLinks
(
    EDMA_Handle             handle,
    DPEDMA_4LinkChanCfg     *chanCfg,
    uint8_t                 chainChId0,
    uint8_t                 chainChId1,
    uint8_t                 chainChId2,
    uint8_t                 chainChId3,
    uint16_t                numIter
);

static int32_t rangeProcHWA_ParseConfig
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_Config  *pConfigIn
);

static int32_t rangeProcHWA_HardwareConfig
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
);

/**************************************************************************
 ************************RangeProcHWA Internal Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 *
 *  @param[in]  threadIdx           Thread index
 *  @param[in]  arg                 Argument to the callback function
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
volatile uint32_t hwadoneisr = 0;
static void rangeProcHWADoneIsrCallback(void * arg)
{
    hwadoneisr++;
    if (arg != NULL) 
    {
        SemaphoreP_post((SemaphoreP_Object*)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA processing completion call back function as per EDMA API.
 *
 *  @param[in]  arg                     Argument to the callback function
 *  @param[in]  transferCompletionCode  EDMA transfer complete code
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
volatile uint32_t EdmaCallbackcnt = 0;
void rangeProcHWA_EDMA_transferCompletionCallbackFxn(Edma_IntrHandle intrHandle,
   void *args)
{
    rangeProcHWAObj     *rangeProcObj;

    /* Get rangeProc object */
    rangeProcObj = (rangeProcHWAObj *)args;

    EdmaCallbackcnt++;
    if (intrHandle->tccNum == rangeProcObj->dataOutSignatureChan)
    {
        rangeProcObj->numEdmaDataOutCnt++;
        SemaphoreP_post(&rangeProcObj->edmaDoneSemaHandle);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to config a dummy channel with 2 linked paramset. Each paramset is linked
 *   to a EDMA data copy channel
 *
 *  @param[in]  handle                  EDMA handle
 *  @param[in]  chanCfg                 EDMA channel configuraton
 *  @param[in]  chainChId0              linked EDMA channel 1
 *  @param[in]  chainChId1              linked EDMA channel 2
 *  @param[in]  numIter                 Number of iterations the dummy channel will be excuted.
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
static int32_t rangeProcHWA_ConfigEDMADummyTwoLinks
(
    EDMA_Handle             handle,
    DPEDMA_4LinkChanCfg     *chanCfg,
    uint8_t                 chainChId0,
    uint8_t                 chainChId1,
    uint16_t                numIter
)
{
    EDMACCPaRAMEntry   edmaParam;
    int32_t             errorCode = SystemP_SUCCESS;
    uint16_t            linkChId0;
    uint16_t            linkChId1;
    uint32_t            baseAddr, regionId;

    baseAddr = EDMA_getBaseAddr(handle);
    if(baseAddr == 0)
    {
        goto exit;
    }

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Get LinkChan from configuraiton */
    linkChId0 = chanCfg->ShadowPramId[0];
    linkChId1 = chanCfg->ShadowPramId[1];

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) NULL;
    edmaParam.destAddr      = (uint32_t) NULL;
    edmaParam.aCnt          = (uint16_t) 0u;
    edmaParam.bCnt          = (uint16_t) numIter;
    edmaParam.cCnt          = (uint16_t) 0u;
    edmaParam.bCntReload    = (uint16_t) 0u;
    edmaParam.srcBIdx       = (int16_t) 0u;
    edmaParam.destBIdx      = (int16_t) 0u;
    edmaParam.srcCIdx       = (int16_t) 0u;
    edmaParam.destCIdx      = (int16_t) 0u;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.opt          |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMASetPaRAM(baseAddr, chanCfg->paramId, &edmaParam);

    EDMAEnableTransferRegion(baseAddr, regionId, chanCfg->channel, EDMA_TRIG_MODE_EVENT);

    CacheP_wbAll(CacheP_TYPE_ALLD);

    /* Change the parameter set to use different transferCompletionCode */
    {
        EDMACCPaRAMEntry paramConfig;

        EDMACCPaRAMEntry_init(&paramConfig);
        memcpy((void *)&paramConfig, (void *)&edmaParam, sizeof(EDMACCPaRAMEntry));

        /* to #1 EDMA channel */
        paramConfig.opt = 0;
        paramConfig.opt |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

         EDMASetPaRAM(baseAddr, linkChId1, &paramConfig);

        /* to #0 EDMA channel */
        paramConfig.opt = 0u;
        paramConfig.opt |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

         EDMASetPaRAM(baseAddr, linkChId0, &paramConfig);

        /* Link 2 shadow links */
        /* Do not use LinkChannel API, it changes to ParamId's TCC  */
        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)linkChId0), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId1));

        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)linkChId1), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId0));

        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)chanCfg->paramId), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId1));
    }
exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      Function to config a dummy channel with 4 linked paramset. Each paramset is linked
 *   to a EDMA data copy channel
 *
 *  @param[in]  handle                  EDMA handle
 *  @param[in]  chanCfg                 EDMA channel configuraton
 *  @param[in]  chainChId0              linked EDMA channel 1
 *  @param[in]  chainChId1              linked EDMA channel 2
 *  @param[in]  chainChId2              linked EDMA channel 3
 *  @param[in]  chainChId3              linked EDMA channel 4
 *  @param[in]  numIter                 Number of iterations the dummy channel will be excuted.
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval     N/A
 */
static int32_t rangeProcHWA_ConfigEDMADummyFourLinks
(
    EDMA_Handle             handle,
    DPEDMA_4LinkChanCfg     *chanCfg,
    uint8_t                 chainChId0,
    uint8_t                 chainChId1,
    uint8_t                 chainChId2,
    uint8_t                 chainChId3,
    uint16_t                numIter
)
{
    EDMACCPaRAMEntry   edmaParam;
    int32_t             errorCode = SystemP_SUCCESS;
    uint16_t            linkChId0;
    uint16_t            linkChId1;
    uint16_t            linkChId2;
    uint16_t            linkChId3;
    uint32_t            baseAddr, regionId;

    baseAddr = EDMA_getBaseAddr(handle);
    if(baseAddr == 0)
    {
        goto exit;
    }

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Get LinkChan from configuraiton */
    linkChId0 = chanCfg->ShadowPramId[0];
    linkChId1 = chanCfg->ShadowPramId[1];
    linkChId2 = chanCfg->ShadowPramId[2];
    linkChId3 = chanCfg->ShadowPramId[3];

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) NULL;
    edmaParam.destAddr      = (uint32_t) NULL;
    edmaParam.aCnt          = (uint16_t) 0u;
    edmaParam.bCnt          = (uint16_t) numIter;
    edmaParam.cCnt          = (uint16_t) 0u;
    edmaParam.bCntReload    = (uint16_t) 0u;
    edmaParam.srcBIdx       = (int16_t) 0u;
    edmaParam.destBIdx      = (int16_t) 0u;
    edmaParam.srcCIdx       = (int16_t) 0u;
    edmaParam.destCIdx      = (int16_t) 0u;
    edmaParam.linkAddr      = 0xFFFFU;
    edmaParam.opt          |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMASetPaRAM(baseAddr, chanCfg->paramId, &edmaParam);

    EDMAEnableTransferRegion(baseAddr, regionId, chanCfg->channel, EDMA_TRIG_MODE_EVENT);

    CacheP_wbAll(CacheP_TYPE_ALLD); //TODO: is this required?

    /* Change the parameter set to use different transferCompletionCode */
    {
        EDMACCPaRAMEntry paramConfig;

        EDMACCPaRAMEntry_init(&paramConfig);

        memcpy((void *)&paramConfig, (void *)&edmaParam, sizeof(EDMACCPaRAMEntry));

        /* to #3 EDMA channel */
        paramConfig.opt = 0;
        paramConfig.opt |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId3) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

         EDMASetPaRAM(baseAddr, linkChId3, &paramConfig);

        /* to #2 EDMA channel */
        paramConfig.opt = 0;
        paramConfig.opt |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId2) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

         EDMASetPaRAM(baseAddr, linkChId2, &paramConfig);

        /* to #1 EDMA channel */
        paramConfig.opt  = 0U;
        paramConfig.opt |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

         EDMASetPaRAM(baseAddr, linkChId1, &paramConfig);

        /* to #0 EDMA channel */
        paramConfig.opt  = 0U;
        paramConfig.opt |= (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
         ((((uint32_t)chainChId0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

         EDMASetPaRAM(baseAddr, linkChId0, &paramConfig);

        /* Link 4 shadow links */
        /* Do not use LinkChannel API, it changes toParamId's TCC  */
        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)linkChId0), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId1));
        
        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)linkChId1), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId2));

        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)linkChId2), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId3));

        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)linkChId3), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId0));

        HW_WR_FIELD32(baseAddr + EDMA_TPCC_LNK((uint32_t)chanCfg->paramId), EDMA_TPCC_LNK_LINK,
            baseAddr + EDMA_TPCC_OPT((uint32_t)linkChId1));
    }

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform range FFT
 *
 *  @param[in]  rangeProcObj                  Pointer to rangeProc object
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigHWACommon
(
    rangeProcHWAObj     *rangeProcObj
)
{
    HWA_CommonConfig    hwaCommonConfig;
    rangeProc_dpParams  *DPParams;
    int32_t             retVal;

    DPParams = &rangeProcObj->params;

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                               HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                               HWA_COMMONCONFIG_MASK_LFSRSEED;
    
    if (rangeProcObj->hwaCfg.dataInputMode == DPU_RangeProcHWA_InputMode_ISOLATED)
    {
        /* HWA will input data from M0 memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    }
    else
    {
        /* HWA will input data from ADC buffer memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
    }
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    hwaCommonConfig.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaCommonConfig.fftConfig.lfsrSeed = 0x1234567; /*Some non-zero value*/
    hwaCommonConfig.numLoops = DPParams->numChirpsPerFrame/2U; //ping/pong path
    hwaCommonConfig.paramStartIdx = rangeProcObj->hwaCfg.paramSetStartIdx;
    hwaCommonConfig.paramStopIdx = rangeProcObj->hwaCfg.paramSetStartIdx + rangeProcObj->hwaCfg.numParamSet - 1U;

    retVal = HWA_configCommon(rangeProcObj->initParms.hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(rangeProcObj->initParms.hwaHandle,
                                        rangeProcHWADoneIsrCallback,
                                        (void*)&rangeProcObj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Trigger HWA for range processing.
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_TriggerHWA
(
    rangeProcHWAObj     *rangeProcObj
)
{
    int32_t             retVal = 0;
    HWA_Handle          hwaHandle;

    /* Get HWA driver handle */
    hwaHandle = rangeProcObj->initParms.hwaHandle;

    /* Configure HWA common parameters */
    retVal = rangeProcHWA_ConfigHWACommon(rangeProcObj);
    if(retVal < 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(hwaHandle, 1);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger the HWA paramset for Ping */
    retVal = HWA_setDMA2ACCManualTrig(hwaHandle, rangeProcObj->dataOutTrigger[0]);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger the HWA paramset for Pong */
    retVal = HWA_setDMA2ACCManualTrig(hwaHandle, rangeProcObj->dataOutTrigger[1]);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to parse rangeProc configuration and save in internal rangeProc object
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *  @param[in]  pConfigIn                 Pointer to rangeProcHWA configuration structure
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ParseConfig
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_Config  *pConfigIn
)
{
    int32_t                 retVal = 0;
    rangeProc_dpParams      *params;
    DPU_RangeProcHWA_StaticConfig   *pStaticCfg;

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    params    = &rangeProcObj->params;

    /* Save datapath parameters */
    params->numTxAntennas = pStaticCfg->numTxAntennas;
    params->numRxAntennas = pStaticCfg->ADCBufData.dataProperty.numRxAntennas;
    params->numVirtualAntennas = pStaticCfg->numVirtualAntennas;
    params->numChirpsPerChirpEvent = pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent;
    params->numAdcSamples = pStaticCfg->ADCBufData.dataProperty.numAdcSamples;
    params->numRangeBins = pStaticCfg->numRangeBins;
    params->numFFTBins = pStaticCfg->numFFTBins;

    if(pStaticCfg->isChirpDataReal){
        params->isReal = 1;
        params->sizeOfInputSample = sizeof(int16_t);
    }
    else{
        params->isReal = 0;
        params->sizeOfInputSample = sizeof(cmplx16ImRe_t);
    }
    params->numChirpsPerFrame = pStaticCfg->numChirpsPerFrame;
    params->numDopplerChirps = pStaticCfg->numChirpsPerFrame/pStaticCfg->numTxAntennas;
    params->fftOutputDivShift = pStaticCfg->rangeFFTtuning.fftOutputDivShift;
    params->numLastButterflyStagesToScale = pStaticCfg->rangeFFTtuning.numLastButterflyStagesToScale;

    /* Save buffers */
    rangeProcObj->ADCdataBuf        = (cmplx16ImRe_t *)pStaticCfg->ADCBufData.data;


    rangeProcObj->radarCubebuf      = (cmplx16ImRe_t *)pConfigIn->hwRes.radarCube.data;

    /* Save interleave mode from ADCBuf configuraiton */
    rangeProcObj->interleave = pStaticCfg->ADCBufData.dataProperty.interleave;

    if((rangeProcObj->interleave ==DPIF_RXCHAN_NON_INTERLEAVE_MODE) &&
        (rangeProcObj->params.numRxAntennas >= 1) )
    {
        /* For rangeProcDPU needs rx channel has same offset from one channel to the next channel
           Use first two channel offset to calculate the BIdx for EDMA
         */
        rangeProcObj->rxChanOffset = pStaticCfg->ADCBufData.dataProperty.rxChanOffset[1] - 
                                    pStaticCfg->ADCBufData.dataProperty.rxChanOffset[0];

        /* rxChanOffset should be 16 bytes aligned and should be big enough to hold numAdcSamples */
        if ((rangeProcObj->rxChanOffset < (rangeProcObj->params.numAdcSamples * rangeProcObj->params.sizeOfInputSample)) ||
          ((rangeProcObj->rxChanOffset & 0xF) != 0))
        {
            retVal = DPU_RANGEPROCHWA_EADCBUF_INTF;
            goto exit;
        }
    }

    /* Save RadarCube format */
    if (pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_2)
    {
        rangeProcObj->radarCubeLayout = rangeProc_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt;
    }
    else if(pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_7)
    {
        rangeProcObj->radarCubeLayout = rangeProc_dataLayout_RANGE_TxAnt_DOPPLER_RxAnt;
    }
    else
    {
        retVal = DPU_RANGEPROCHWA_EINTERNAL;
        goto exit;
    }

    /* Prepare internal hardware resouces = trigger source matchs its  paramset index */
    rangeProcObj->dataInTrigger[0]      = 1U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;
    rangeProcObj->dataInTrigger[1]      = 3U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;
    rangeProcObj->dataOutTrigger[0]     = 0U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;
    rangeProcObj->dataOutTrigger[1]     = 2U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;

    /* Save hardware resources that will be used at runtime */
    rangeProcObj->edmaHandle= pConfigIn->hwRes.edmaHandle;
    rangeProcObj->dataOutSignatureChan = pConfigIn->hwRes.edmaOutCfg.dataOutSignature.tcc;
    memcpy((void *)&rangeProcObj->hwaCfg, (void *)&pConfigIn->hwRes.hwaCfg, sizeof(DPU_RangeProcHWA_HwaConfig));

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      EDMA configuration for rangeProc data in when EDMA is used to copy data from 
 *  ADCBuf to HWA memory
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object handle
 *  @param[in]  DPParams                  Pointer to datapath parameter
 *  @param[in]  pHwConfig                 Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigEDMA_DataIn
(
    rangeProcHWAObj         *rangeProcObj,
    rangeProc_dpParams      *DPParams,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    int32_t             errorCode = SystemP_SUCCESS;
    EDMA_Handle         handle ;
    uint16_t            bytePerRxChan;
    DPEDMA_ChainingCfg  chainingCfg;

    /* Get rangeProc Configuration */
    handle = rangeProcObj->edmaHandle;

    bytePerRxChan = DPParams->numAdcSamples * DPParams->sizeOfInputSample;

    /**********************************************/
    /* ADCBuf -> Ping/Pong Buffer(M0 and M1)           */
    /**********************************************/
    chainingCfg.chainingChan = pHwConfig->edmaInCfg.dataInSignature.channel;
    chainingCfg.isFinalChainingEnabled = true;
    chainingCfg.isIntermediateChainingEnabled = true;

    DPEDMA_syncABCfg    syncABCfg;

    syncABCfg.srcAddress = (uint32_t)rangeProcObj->ADCdataBuf;
    syncABCfg.destAddress = rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PING];

    syncABCfg.aCount = bytePerRxChan;
    syncABCfg.bCount = DPParams->numRxAntennas;
    syncABCfg.cCount =2U; /* ping and pong */

    syncABCfg.srcBIdx=rangeProcObj->rxChanOffset;
    syncABCfg.dstBIdx=rangeProcObj->rxChanOffset;
    syncABCfg.srcCIdx=0U;
    
    syncABCfg.dstCIdx=((uint32_t)rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PONG] - (uint32_t)rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_SRC_PING]);

    errorCode = DPEDMA_configSyncAB(handle,
                                    &pHwConfig->edmaInCfg.dataIn,
                                    &chainingCfg,
                                    &syncABCfg,
                                    true,    /* isEventTriggered */
                                    /* Intermediate and Final transfer interrupts are enabled in case
                                        * the user wants to poll the IPR(H) register (for example, 
                                        * in the range proc test case) or register an ISR for when a chirp 
                                        * transfer from to HWA memory is complete */
                                    true,  /* isIntermediateTransferCompletionEnabled */
                                    true,   /* isTransferCompletionEnabled */
                                    NULL,
                                    NULL,
                                    NULL);

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

    /*************************************************/
    /* Generate Hot Signature to trigger Ping/Pong paramset   */
    /*************************************************/

    errorCode = DPEDMAHWA_configTwoHotSignature(handle, 
                                                  &pHwConfig->edmaInCfg.dataInSignature,
                                                  rangeProcObj->initParms.hwaHandle,
                                                  rangeProcObj->dataInTrigger[0],
                                                  rangeProcObj->dataInTrigger[1],
                                                  false);

    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }
exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform range FFT
 *
 *  @param[in]  rangeProcObj                  Pointer to rangeProc object
 *  @param[in]  destChanPing                  Destination channel id for PING
 *  @param[in]  destChanPong                  Destination channel id for PONG
 *  @param[in]  hwaMemSrcPingOffset           Source Address offset for Ping input
 *  @param[in]  hwaMemSrcPongOffset           Source Address offset for Pong input
 *  @param[in]  hwaMemDestPingOffset          Destination address offset for Ping output
 *  @param[in]  hwaMemDestPongOffset          Destination address offset for Pong output
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigHWA
(
    rangeProcHWAObj     *rangeProcObj,
    uint8_t     destChanPing,
    uint8_t     destChanPong,
    uint32_t    hwaMemSrcPingOffset,
    uint32_t    hwaMemSrcPongOffset,
    uint32_t    hwaMemDestPingOffset,
    uint32_t    hwaMemDestPongOffset
)
{
    HWA_InterruptConfig     paramISRConfig;
    int32_t                 errCode = 0;
    uint32_t                paramsetIdx = 0;
    uint32_t                hwParamsetIdx;
    uint32_t                pingParamSetIdx = 0;
    HWA_ParamConfig         hwaParamCfg[DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS];
    HWA_Handle                      hwaHandle;
    rangeProc_dpParams             *pDPParams;
    uint8_t                         index;

    hwaHandle = rangeProcObj->initParms.hwaHandle;
    pDPParams = &rangeProcObj->params;

    memset(hwaParamCfg,0,sizeof(hwaParamCfg));

    hwParamsetIdx = rangeProcObj->hwaCfg.paramSetStartIdx;
    for(index = 0; index < DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS; index++)
    {
        errCode = HWA_disableParamSetInterrupt(hwaHandle, index + rangeProcObj->hwaCfg.paramSetStartIdx, 
                HWA_PARAMDONE_INTERRUPT_TYPE_CPU |HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
        if (errCode != 0)
        {
            goto exit;
        }
    }

    /***********************/
    /* PING DUMMY PARAMSET */
    /***********************/
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE;
    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    /***********************/
    /* PING PROCESS PARAMSET */
    /***********************/
    paramsetIdx++;
    hwParamsetIdx++;
    pingParamSetIdx = paramsetIdx;

    /* adcbuf not mapped, HWA is triggered after edma copy is done */
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;

    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemSrcPingOffset; 

    if(pDPParams->isReal){
        hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    }
    else{
        hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    }

    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0;
    hwaParamCfg[paramsetIdx].source.srcScale = 8;

    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemDestPingOffset; 

    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; 
    hwaParamCfg[paramsetIdx].dest.dstScale = pDPParams->fftOutputDivShift;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; 

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(pDPParams->numFFTBins);
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 
                                    (1 << pDPParams->numLastButterflyStagesToScale) - 1U;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = rangeProcObj->hwaCfg.hwaWinRamOffset; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = rangeProcObj->hwaCfg.hwaWinSym; 

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /* HWA range FFT src/dst configuration*/

    hwaParamCfg[paramsetIdx].source.srcAcnt = pDPParams->numAdcSamples - 1;
    hwaParamCfg[paramsetIdx].source.srcAIdx = pDPParams->sizeOfInputSample;
    hwaParamCfg[paramsetIdx].source.srcBcnt = pDPParams->numRxAntennas-1;
    hwaParamCfg[paramsetIdx].source.srcBIdx = rangeProcObj->rxChanOffset;

    hwaParamCfg[paramsetIdx].dest.dstAcnt = pDPParams->numRangeBins-1;
    hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint32_t) * pDPParams->numRxAntennas; 
    hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t);

    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChanPing;

    errCode = HWA_enableParamSetInterrupt(hwaHandle,hwParamsetIdx,&paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }

    /***********************/
    /* PONG DUMMY PARAMSET */
    /***********************/
    paramsetIdx++;
    hwParamsetIdx++;

    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE;
    errCode = HWA_configParamSet(hwaHandle, 
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    /***********************/
    /* PONG PROCESS PARAMSET */
    /***********************/
    paramsetIdx++;
    hwParamsetIdx++;
    hwaParamCfg[paramsetIdx] = hwaParamCfg[pingParamSetIdx];
    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemSrcPongOffset; 
    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemDestPongOffset;

    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;

    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    /* Enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChanPong;
    errCode = HWA_enableParamSetInterrupt(hwaHandle, 
                                           hwParamsetIdx,
                                           &paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }
exit:
    return(errCode);
}

/**
 *  @b Description
 *  @n
 *      EDMA configuration for rangeProc data output in non-interleave mode
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *  @param[in]  DPParams                  Pointer to datapath parameter
 *  @param[in]  pHwConfig                 Pointer to rangeProc hardware resources
 *  @param[in]  hwaOutPingOffset          Ping HWA memory address offset
 *  @param[in]  hwaOutPongOffset          Pong HWA memory address offset
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_ConfigEDMA_DataOut
(
    rangeProcHWAObj         *rangeProcObj,
    rangeProc_dpParams      *DPParams,
    DPU_RangeProcHWA_HW_Resources *pHwConfig,
    uint32_t                hwaOutPingOffset,
    uint32_t                hwaOutPongOffset
)
{
    int32_t                 errorCode = SystemP_SUCCESS;
    EDMA_Handle             handle;
    DPEDMA_syncABCfg        syncABCfg;
    DPEDMA_ChainingCfg      chainingCfg;

    /* Get rangeProc Configuration */
    handle = rangeProcObj->edmaHandle;

    /* Chaining configuration for all cases -> chaining to the data out signature channel */
    chainingCfg.chainingChan = pHwConfig->edmaOutCfg.dataOutSignature.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled = true;

     /**************************************************************************
      *  Configure EDMA to copy from HWA memory to radar cube 
      *************************************************************************/
    if(rangeProcObj->radarCubeLayout == rangeProc_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt)
    {
        /* Ping/Pong common configuration */
        syncABCfg.aCount = DPParams->numRxAntennas * sizeof(uint32_t);
        syncABCfg.bCount = DPParams->numRangeBins;
        syncABCfg.cCount = DPParams->numChirpsPerFrame/2U;
        syncABCfg.srcBIdx = DPParams->numRxAntennas * sizeof(uint32_t);
        syncABCfg.srcCIdx = 0U;
        syncABCfg.dstBIdx = DPParams->numRxAntennas * sizeof(uint32_t) *DPParams->numChirpsPerFrame;
        syncABCfg.dstCIdx = DPParams->numRxAntennas * 2U * sizeof(uint32_t);

        /* Ping specific config */
        syncABCfg.srcAddress = hwaOutPingOffset;
        syncABCfg.destAddress= (uint32_t)rangeProcObj->radarCubebuf;

        errorCode = DPEDMA_configSyncAB(handle,
                &pHwConfig->edmaOutCfg.u.fmt2.dataOutPing,
                &chainingCfg,
                &syncABCfg,
                true,    /* isEventTriggered */
                false,   /* isIntermediateTransferCompletionEnabled */
                false,   /* isTransferCompletionEnabled */
                NULL,
                NULL,
                pHwConfig->intrObj);

        if (errorCode != SystemP_SUCCESS)
        {
            goto exit;
        }

        /* Pong specific configuration */
        syncABCfg.srcAddress = hwaOutPongOffset;
        syncABCfg.destAddress= (uint32_t)(rangeProcObj->radarCubebuf + DPParams->numRxAntennas);

        errorCode = DPEDMA_configSyncAB(handle,
                &pHwConfig->edmaOutCfg.u.fmt2.dataOutPong,
                &chainingCfg,
                &syncABCfg,
                true,   /* isEventTriggered */
                false,  /* isIntermediateTransferCompletionEnabled */
                true,   /* isTransferCompletionEnabled */
                rangeProcHWA_EDMA_transferCompletionCallbackFxn,
                (void*)rangeProcObj,
                pHwConfig->intrObj);
        if (errorCode != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else
    {
        uint32_t    numSamplePerRbin;

        numSamplePerRbin = DPParams->numRxAntennas   * DPParams->numDopplerChirps;

        if (DPParams->numTxAntennas == 4U)
        {

            /**************************************************************************
            *  Configure EDMA to copy HWA results to radar cube 
            *  For cases with 4 TX Antenna
            // *************************************************************************/
            DPEDMA_syncABCfg    syncABCfg_fmt7;
            uint32_t    destAddr[2][2];
            uint8_t     index;
            bool        lastChan = false;

            destAddr[0][0] = (uint32_t)rangeProcObj->radarCubebuf;
            destAddr[0][1] = (uint32_t)(rangeProcObj->radarCubebuf + 2 * numSamplePerRbin);
            destAddr[1][0] = (uint32_t)(rangeProcObj->radarCubebuf + numSamplePerRbin);
            destAddr[1][1] = (uint32_t)(rangeProcObj->radarCubebuf + 3 * numSamplePerRbin);

            /* Desitnation EDMA */
            /* Ping configuration to transfer 1D FFT output from HWA to L3 RAM */
            errorCode = rangeProcHWA_ConfigEDMADummyTwoLinks(handle,
                &pHwConfig->edmaOutCfg.u.fmt7.dataOutPing,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[0].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[1].channel,
                DPParams->numChirpsPerFrame/2U
                );
            if (errorCode != SystemP_SUCCESS)
            {
                goto exit;
            }

            /* Desitnation EDMA */
            /* Pong configuration to transfer 1D FFT output from HWA to L3 RAM */
            errorCode = rangeProcHWA_ConfigEDMADummyTwoLinks(handle,
                &pHwConfig->edmaOutCfg.u.fmt7.dataOutPong,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPongData[0].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPongData[1].channel,
                DPParams->numChirpsPerFrame/2U
                );
            if (errorCode != SystemP_SUCCESS)
            {
                goto exit;
            }

            /* Ping/Pong common configuration */
            syncABCfg_fmt7.aCount = DPParams->numRxAntennas * sizeof(uint32_t);
            syncABCfg_fmt7.bCount = DPParams->numRangeBins;
            syncABCfg_fmt7.cCount = DPParams->numChirpsPerFrame/4U;
            syncABCfg_fmt7.srcBIdx = DPParams->numRxAntennas * sizeof(uint32_t);
            syncABCfg_fmt7.srcCIdx = 0U;
            syncABCfg_fmt7.dstBIdx = DPParams->numRxAntennas * sizeof(uint32_t) *DPParams->numChirpsPerFrame;
            syncABCfg_fmt7.dstCIdx = DPParams->numRxAntennas * sizeof(uint32_t);

            for(index=0;index < 2; index++)
            {
                if(index == 1)
                {
                    /* Set last channel flag to enable completion flag */
                    lastChan = true;
                }

                /* Configure EDMA channels to copy data from M2 to one of TX antenna radar Cube */
                /* PING specific config 
                   M2 - >Txi (i=0,2) */

                syncABCfg_fmt7.srcAddress = hwaOutPingOffset;
                syncABCfg_fmt7.destAddress = destAddr[0][index];

                errorCode = DPEDMA_configSyncAB(handle,
                            &pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[index],
                            &chainingCfg,
                            &syncABCfg_fmt7,
                            false,    /* isEventTriggered */
                            false,   /* isIntermediateTransferCompletionEnabled */
                            lastChan,   /* isTransferCompletionEnabled */
                            NULL,
                            NULL,
                            pHwConfig->intrObj);
                
                if (errorCode != SystemP_SUCCESS)
                {
                    goto exit;
                }

                /* PONG specific config 
                   M3 - >Txi (i=1,3) */
                syncABCfg_fmt7.srcAddress = hwaOutPongOffset;
                syncABCfg_fmt7.destAddress= destAddr[1][index];

                errorCode = DPEDMA_configSyncAB(handle,
                            &pHwConfig->edmaOutCfg.u.fmt7.dataOutPongData[index],
                            &chainingCfg,
                            &syncABCfg_fmt7,
                            false,    /* isEventTriggered */
                            false,   /* isIntermediateTransferCompletionEnabled */
                            lastChan,   /* isTransferCompletionEnabled */
                            (lastChan ==true)? rangeProcHWA_EDMA_transferCompletionCallbackFxn: NULL,
                            (lastChan ==true)? (void*)rangeProcObj:NULL,
                            pHwConfig->intrObj);

                if (errorCode != SystemP_SUCCESS)
                {
                    goto exit;
                }
            }
        }
        if (DPParams->numTxAntennas == 8U)
        {
            /**************************************************************************
            *  Configure EDMA to copy HWA results to radar cube 
            *  For cases with 8 TX Antenna
            // *************************************************************************/
            DPEDMA_syncABCfg    syncABCfg_fmt7;
            uint32_t    destAddr[2][4];
            uint8_t     index;
            bool        lastChan = false;

            destAddr[0][0] = (uint32_t)rangeProcObj->radarCubebuf;
            destAddr[0][1] = (uint32_t)(rangeProcObj->radarCubebuf + 2 * numSamplePerRbin);
            destAddr[0][2] = (uint32_t)(rangeProcObj->radarCubebuf + 4 * numSamplePerRbin);
            destAddr[0][3] = (uint32_t)(rangeProcObj->radarCubebuf + 6 * numSamplePerRbin);
            destAddr[1][0] = (uint32_t)(rangeProcObj->radarCubebuf + numSamplePerRbin);
            destAddr[1][1] = (uint32_t)(rangeProcObj->radarCubebuf + 3 * numSamplePerRbin);
            destAddr[1][2] = (uint32_t)(rangeProcObj->radarCubebuf + 5 * numSamplePerRbin);
            destAddr[1][3] = (uint32_t)(rangeProcObj->radarCubebuf + 7 * numSamplePerRbin);

            /* Desitnation EDMA */
            /* Ping configuration to transfer 1D FFT output from HWA to L3 RAM */
            errorCode = rangeProcHWA_ConfigEDMADummyFourLinks(handle,
                &pHwConfig->edmaOutCfg.u.fmt7.dataOutPing,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[0].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[1].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[2].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[3].channel,
                DPParams->numChirpsPerFrame/2U
                );
            if (errorCode != SystemP_SUCCESS)
            {
                goto exit;
            }

            /* Desitnation EDMA */
            /* Pong configuration to transfer 1D FFT output from HWA to L3 RAM */
            errorCode = rangeProcHWA_ConfigEDMADummyFourLinks(handle,
                &pHwConfig->edmaOutCfg.u.fmt7.dataOutPong,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPongData[0].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPongData[1].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[2].channel,
                pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[3].channel,
                DPParams->numChirpsPerFrame/2U
                );
            if (errorCode != SystemP_SUCCESS)
            {
                goto exit;
            }

            /* Ping/Pong common configuration */
            syncABCfg_fmt7.aCount = DPParams->numRxAntennas * sizeof(uint32_t);
            syncABCfg_fmt7.bCount = DPParams->numRangeBins;
            syncABCfg_fmt7.cCount = DPParams->numChirpsPerFrame/8U;
            syncABCfg_fmt7.srcBIdx = DPParams->numRxAntennas * sizeof(uint32_t);
            syncABCfg_fmt7.srcCIdx = 0U;
            syncABCfg_fmt7.dstBIdx = DPParams->numRxAntennas * sizeof(uint32_t) *DPParams->numChirpsPerFrame;
            syncABCfg_fmt7.dstCIdx = DPParams->numRxAntennas * sizeof(uint32_t);

            for(index=0;index < 4; index++)
            {
                if(index == 1)
                {
                    /* Set last channel flag to enable completion flag */
                    lastChan = true;
                }

                /* Configure EDMA channels to copy data from M2 to one of TX antenna radar Cube */
                /* PING specific config 
                   M2 - >Txi (i=0,2,4,6) */

                syncABCfg_fmt7.srcAddress = hwaOutPingOffset;
                syncABCfg_fmt7.destAddress = destAddr[0][index];

                errorCode = DPEDMA_configSyncAB(handle,
                            &pHwConfig->edmaOutCfg.u.fmt7.dataOutPingData[index],
                            &chainingCfg,
                            &syncABCfg_fmt7,
                            false,    /* isEventTriggered */
                            false,   /* isIntermediateTransferCompletionEnabled */
                            lastChan,   /* isTransferCompletionEnabled */
                            NULL,
                            NULL,
                            pHwConfig->intrObj);
                
                if (errorCode != SystemP_SUCCESS)
                {
                    goto exit;
                }

                /* PONG specific config 
                   M3 - >Txi (i=1,3,5,7) */
                syncABCfg_fmt7.srcAddress = hwaOutPongOffset;
                syncABCfg_fmt7.destAddress= destAddr[1][index];

                errorCode = DPEDMA_configSyncAB(handle,
                            &pHwConfig->edmaOutCfg.u.fmt7.dataOutPongData[index],
                            &chainingCfg,
                            &syncABCfg_fmt7,
                            false,    /* isEventTriggered */
                            false,   /* isIntermediateTransferCompletionEnabled */
                            lastChan,   /* isTransferCompletionEnabled */
                            (lastChan ==true)? rangeProcHWA_EDMA_transferCompletionCallbackFxn: NULL,
                            (lastChan ==true)? (void*)rangeProcObj:NULL,
                            pHwConfig->intrObj);

                if (errorCode != SystemP_SUCCESS)
                {
                    goto exit;
                }
            }

        }
    }

    /**************************************************************************
    *  HWA hot signature EDMA, chained to the transpose EDMA channels
    *************************************************************************/
    errorCode = DPEDMAHWA_configTwoHotSignature(handle, 
                                                  &pHwConfig->edmaOutCfg.dataOutSignature,
                                                  rangeProcObj->initParms.hwaHandle,
                                                  rangeProcObj->dataOutTrigger[0],
                                                  rangeProcObj->dataOutTrigger[1],
                                                  false);
    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      rangeProc configuration in non-interleaved mode
 *
 *  @param[in]  rangeProcObj                 Pointer to rangeProc object
 *  @param[in]  DPParams                     Pointer to data path common params
 *  @param[in]  pHwConfig                    Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_Config
(
    rangeProcHWAObj          *rangeProcObj,
    rangeProc_dpParams       *DPParams,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    HWA_Handle          hwaHandle;
    int32_t             retVal = 0;
    uint8_t             destChanPing;
    uint8_t             destChanPong;
    uint8_t             edmaChanPing;
    uint8_t             edmaChanPong;

    hwaHandle = rangeProcObj->initParms.hwaHandle;


    if((rangeProcObj->radarCubeLayout == rangeProc_dataLayout_RANGE_TxAnt_DOPPLER_RxAnt) &&
        (DPParams->numTxAntennas == 8U || DPParams->numTxAntennas == 4U) )
    {
        edmaChanPing = pHwConfig->edmaOutCfg.u.fmt7.dataOutPing.channel;
        edmaChanPong = pHwConfig->edmaOutCfg.u.fmt7.dataOutPong.channel;
    }
    else
    {
        edmaChanPing = pHwConfig->edmaOutCfg.u.fmt2.dataOutPing.channel;
        edmaChanPong = pHwConfig->edmaOutCfg.u.fmt2.dataOutPong.channel;
    }

    /* Get HWA destination channel id */
    retVal = HWA_getDMAChanIndex(hwaHandle, edmaChanPing, &destChanPing);
    if (retVal != 0)
    {
        goto exit;
    }
    
    retVal = HWA_getDMAChanIndex(hwaHandle, edmaChanPong, &destChanPong);
    if (retVal != 0)
    {
        goto exit;
    }

    if(pHwConfig->hwaCfg.dataInputMode == DPU_RangeProcHWA_InputMode_ISOLATED)
    {
        /* Copy data from ADC buffer to HWA buffer */ 
        rangeProcHWA_ConfigEDMA_DataIn(rangeProcObj,    DPParams, pHwConfig);

        /* Range FFT configuration in HWA */
        retVal = rangeProcHWA_ConfigHWA(rangeProcObj,
            destChanPing,
            destChanPong,
            DPU_RANGEHWA_SRCADDR_PING,
            DPU_RANGEHWA_SRCADDR_PONG,
            DPU_RANGEHWA_DSTADDR_PING,
            DPU_RANGEHWA_DSTADDR_PONG
        );
        if (retVal < 0)
        {
            goto exit;
        }

        /* Data output EDMA configuration */
        retVal = rangeProcHWA_ConfigEDMA_DataOut(rangeProcObj,
            DPParams,
            pHwConfig,
            (uint32_t)rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_DST_PING],
            (uint32_t)rangeProcObj->hwaMemBankAddr[DPU_RANGEHWA_MEM_BANK_INDX_DST_PONG]);
        if (retVal < 0)
        {
            goto exit;
        }
    }
    else
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
    }

    if (retVal < 0)
    {
        goto exit;
    }

exit:
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      Internal function to config HWA/EDMA to perform range FFT
 *
 *  @param[in]  rangeProcObj              Pointer to rangeProc object
 *  @param[in]  pHwConfig                 Pointer to rangeProc hardware resources
 *
 *  \ingroup    DPU_RANGEPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t rangeProcHWA_HardwareConfig
(
    rangeProcHWAObj         *rangeProcObj,
    DPU_RangeProcHWA_HW_Resources *pHwConfig
)
{
    int32_t                 retVal = 0;
    rangeProc_dpParams      *DPParams;
    DPParams    = &rangeProcObj->params;

    retVal = rangeProcHWA_Config(rangeProcObj, DPParams, pHwConfig);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**************************************************************************
 ************************RangeProcHWA External APIs **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              Pointer to DPU init parameters
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid rangeProc handle
 *  @retval
 *      Error       - NULL
 */
DPU_RangeProcHWA_Handle DPU_RangeProcHWA_init
(
    DPU_RangeProcHWA_InitParams     *initParams,
    int32_t*                        errCode
)
{
    rangeProcHWAObj     *rangeProcObj = NULL;
    HWA_MemInfo         hwaMemInfo;
    uint8_t             index;
    int32_t             status = SystemP_SUCCESS;

    *errCode = 0;

    if( (initParams == NULL) ||
       (initParams->hwaHandle == NULL) )
    {
        *errCode = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Allocate Memory for rangeProc */
    rangeProcObj = (rangeProcHWAObj*)&RangeObj;

    if(rangeProcObj == NULL)
    {
        *errCode = DPU_RANGEPROCHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)rangeProcObj, 0, sizeof(rangeProcHWAObj));

    memcpy((void *)&rangeProcObj->initParms, initParams, sizeof(DPU_RangeProcHWA_InitParams));

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        rangeProcObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructBinary(&rangeProcObj->edmaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_RANGEPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    status = SemaphoreP_constructBinary(&rangeProcObj->hwaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_RANGEPROCHWA_ESEMA;
        goto exit;
    }

exit:
    if(*errCode < 0)
    {
        rangeProcObj = (DPU_RangeProcHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_RangeProcHWA_Handle)rangeProcObj);

}

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU config function. It saves buffer pointer and configurations 
 *  including system resources and configures HWA and EDMA for runtime range processing.
 *  
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle                  rangeProc DPU handle
 *  @param[in]  pConfigIn               Pointer to rangeProc configuration data structure
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_config
(
    DPU_RangeProcHWA_Handle  handle,
    DPU_RangeProcHWA_Config  *pConfigIn
)
{
    rangeProcHWAObj                 *rangeProcObj;
    DPU_RangeProcHWA_StaticConfig   *pStaticCfg;
    HWA_Handle                      hwaHandle;
    int32_t                         retVal = 0;

    rangeProcObj = (rangeProcHWAObj *)handle;
    if(rangeProcObj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    hwaHandle = rangeProcObj->initParms.hwaHandle;

#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!pConfigIn ||
      !(pConfigIn->hwRes.edmaHandle)
      )
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Parameter check: validate Adc data interface configuration
        Support:
            - 1 chirp per chirpEvent
            - Complex 16bit ADC data in IMRE format
     */

    if((pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_REAL16) ||
    (pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent != 1U) ) 
    {
        retVal = DPU_RANGEPROCHWA_EADCBUF_INTF;
        goto exit;
    }

    /* Parameter check: windowing Size */
    {
        uint16_t expectedWinSize;

        if( pConfigIn->hwRes.hwaCfg.hwaWinSym == HWA_FFT_WINDOW_SYMMETRIC)
        {
            /* Only half of the windowing factor is needed for symmetric window */
            expectedWinSize = ((pStaticCfg->ADCBufData.dataProperty.numAdcSamples + 1U) / 2U ) * sizeof(uint32_t);
        }
        else
        {
            expectedWinSize = pStaticCfg->ADCBufData.dataProperty.numAdcSamples * sizeof(uint32_t);
        }

        if(pStaticCfg->windowSize != expectedWinSize)
        {
            retVal = DPU_RANGEPROCHWA_EWINDOW;
            goto exit;
        }
    }

    /* Refer to radar cube definition for FORMAT_x , the following are the only supported formats
        Following assumption is made upon radar cube FORMAT_x definition 
           1. data type is complex in cmplx16ImRe_t format only
           2. It is always 1D range output.
     */
    if( (pConfigIn->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_2) &&
        (pConfigIn->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_7) )
    {
        retVal = DPU_RANGEPROCHWA_ERADARCUBE_INTF;
        goto exit;
    }

    /* Not supported input & output format combination */
    if (pStaticCfg->ADCBufData.dataProperty.interleave == DPIF_RXCHAN_INTERLEAVE_MODE)
    {
        retVal = DPU_RANGEPROCHWA_ENOTIMPL;
        goto exit;
    }

    /* Parameter check: radarcube buffer Size */
    if (pConfigIn->hwRes.radarCube.dataSize != (pStaticCfg->numRangeBins* sizeof(cmplx16ImRe_t) *
                                      pStaticCfg->numChirpsPerFrame *
                                      pStaticCfg->ADCBufData.dataProperty.numRxAntennas) )
    {
        retVal = DPU_RANGEPROCHWA_ERADARCUBE_INTF;
        goto exit;
    }

    /* Parameter check: Num butterfly stages to scale */
    if (pStaticCfg->rangeFFTtuning.numLastButterflyStagesToScale > mathUtils_ceilLog2(pStaticCfg->numRangeBins))
    {
        retVal = DPU_RANGEPROCHWA_EBUTTERFLYSCALE;
        goto exit;
    }
#endif

    retVal = rangeProcHWA_ParseConfig(rangeProcObj, pConfigIn);
    if (retVal < 0)
    {
        goto exit;
    }

    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Reset the internal state of the HWA */
    retVal = HWA_reset(hwaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Windowing configuraiton in HWA */
    retVal = HWA_configRam(hwaHandle,
                            HWA_RAM_TYPE_WINDOW_RAM,
                            (uint8_t *)pStaticCfg->window,
                            pStaticCfg->windowSize,   /* size in bytes */
                            pConfigIn->hwRes.hwaCfg.hwaWinRamOffset * sizeof(uint32_t));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Clear stats */
    rangeProcObj->numProcess = 0U;

    /* Initial configuration of rangeProc */
    retVal = rangeProcHWA_HardwareConfig(rangeProcObj, &pConfigIn->hwRes);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU process function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle                  rangeProc DPU handle
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_process
(
    DPU_RangeProcHWA_Handle     handle,
    DPU_RangeProcHWA_OutParams  *outParams
)
{
    rangeProcHWAObj     *rangeProcObj;
    int32_t             retVal = 0;

    rangeProcObj = (rangeProcHWAObj *)handle;
    if ((rangeProcObj == NULL) ||
        (outParams == NULL))
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Set inProgress state */
    rangeProcObj->inProgress = true;
    outParams->endOfChirp = false;

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    /* wait for the all paramSets done interrupt */
    SemaphoreP_pend(&rangeProcObj->hwaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /**********************************************/
    /* WAIT FOR EDMA INTERRUPT                    */
    /**********************************************/
    SemaphoreP_pend(&rangeProcObj->edmaDoneSemaHandle, SystemP_WAIT_FOREVER);

    /* Range FFT is done, disable Done interrupt */
    HWA_disableDoneInterrupt(rangeProcObj->initParms.hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(rangeProcObj->initParms.hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Update stats and output parameters */
    rangeProcObj->numProcess++;

    /* Following stats is not available for rangeProcHWA */
    outParams->stats.processingTime = 0;
    outParams->stats.waitTime= 0;

    outParams->endOfChirp = true;

    /* Clear inProgress state */
    rangeProcObj->inProgress = false;

exit:

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU control function. 
 *
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle           rangeProc DPU handle
 *  @param[in]  cmd              rangeProc DPU control command
 *  @param[in]  arg              rangeProc DPU control argument pointer
 *  @param[in]  argSize          rangeProc DPU control argument size
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_control
(
    DPU_RangeProcHWA_Handle handle,
    DPU_RangeProcHWA_Cmd    cmd,
    void*                   arg,
    uint32_t                argSize
)
{
    int32_t             retVal = 0;
    rangeProcHWAObj     *rangeProcObj;

    /* Get rangeProc data object */
    rangeProcObj = (rangeProcHWAObj *)handle;

    /* Sanity check */
    if (rangeProcObj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Check if control() is called during processing time */
    if(rangeProcObj->inProgress == true)
    {
        retVal = DPU_RANGEPROCHWA_EINPROGRESS;
        goto exit;
    }

    /* Control command handling */
    switch(cmd)
    {
        case DPU_RangeProcHWA_Cmd_triggerProc:
            /* Trigger rangeProc in HWA */
            retVal = rangeProcHWA_TriggerHWA( rangeProcObj);
            if(retVal != 0)
            {
                goto exit;
            }
        break;

        default:
            retVal = DPU_RANGEPROCHWA_ECMD;
            break;
    }
exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU deinit function. It frees the resources used for the DPU.
 *
 *  @pre    DPU_RangeProcHWA_init() has been called
 *
 *  @param[in]  handle           rangeProc DPU handle
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_RangeProcHWA_deinit
(
    DPU_RangeProcHWA_Handle     handle
)
{
    rangeProcHWAObj     *rangeProcObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    rangeProcObj = (rangeProcHWAObj *)handle;
    if(rangeProcObj == NULL)
    {
        retVal = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Delete Semaphores */
    SemaphoreP_destruct(&rangeProcObj->edmaDoneSemaHandle);
    SemaphoreP_destruct(&rangeProcObj->hwaDoneSemaHandle);

exit:

    return (retVal);
}