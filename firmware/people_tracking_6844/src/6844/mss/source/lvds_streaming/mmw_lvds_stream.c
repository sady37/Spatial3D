/**
 *   @file  mmw_lvds_stream.c
 *
 *   @brief
 *      Implements LVDS stream functionality.
 *
 *  \par
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
 
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* MMWSDK Include Files. */
#include <common/syscommon.h>
#include <drivers/edma.h>
#include <drivers/cbuff.h>

/* MMWAVE Demo Include Files */
#include <source/mmw_res.h>
#include <source/mmwave_demo_mss.h>

extern MmwDemo_MSS_MCB    gMmwMssMCB;

/**
 *  @b Description
 *  @n
 *      Allocates Shawdow paramset
 */
static void allocateEDMAShadowChannel(EDMA_Handle handle, uint32_t *param)
{
    int32_t             testStatus = SystemP_SUCCESS;

    testStatus = EDMA_allocParam(handle, param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    return;
}

/**
 *  @b Description
 *  @n
 *      Allocates EDMA resource
 */
void allocateEDMAChannel(EDMA_Handle handle,
    uint32_t *dmaCh,
    uint32_t *tcc,
    uint32_t *param
)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    EDMA_Config        *config;
    EDMA_Object        *object;

    config = (EDMA_Config *) handle;
    object = config->object;

    if((object->allocResource.dmaCh[*dmaCh/32] & (1U << *dmaCh%32)) != (1U << *dmaCh%32))
    {
        testStatus = EDMA_allocDmaChannel(handle, dmaCh);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocTcc(handle, tcc);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocParam(handle, param);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        baseAddr = EDMA_getBaseAddr(handle);
        DebugP_assert(baseAddr != 0);

        regionId = EDMA_getRegionId(handle);
        DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

        /* Request channel */
        EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, *tcc, *param, 0);
   }

    return;
}

 /**
 *  @b Description
 *  @n
 *      This is the LVDS streaming init function. 
 *      It initializes the necessary modules
 *      that implement the streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamInit (void)
{
    CBUFF_InitCfg           initCfg;
    int32_t                 retVal = MINUS_ONE;
    int32_t                 errCode;

    /*************************************************************************************
     * Open the CBUFF Driver:
     *************************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.enableECC                 = 0U;
    initCfg.crcEnable                 = 1U;
    initCfg.maxSessions               = 2U;
    initCfg.enableDebugMode           = false;
    initCfg.interface                 = CBUFF_Interface_LVDS;
    initCfg.outputDataFmt             = CBUFF_OutputDataFmt_16bit;
    initCfg.lvdsCfg.crcEnable         = 0U;
    initCfg.lvdsCfg.msbFirst          = 1U;
    /* Enable all lanes available on the platform*/
    initCfg.lvdsCfg.lvdsLaneEnable    = 0x3U;
    initCfg.lvdsCfg.ddrClockMode      = 1U;
    initCfg.lvdsCfg.ddrClockModeMux   = 1U;

    /* Provide chirp_avail IRQ map to CBUFF to initiate transfer */
    HW_WR_REG32(CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_CBUFF_TRIGGER_SEL, CSL_DSS_INTR_FECSS_CHIRP_AVAIL_IRQ);

    /* Initialize the CBUFF Driver: */
    gMmwMssMCB.lvdsStream.cbuffHandle = CBUFF_open (&initCfg, &errCode);
    if (gMmwMssMCB.lvdsStream.cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF Driver */
        CLI_write("Error: CBUFF_open failed with [Error=%d]\n", errCode);
        goto exit;
    }
    
    gMmwMssMCB.lvdsStream.sessionEDMAChannelAllocatorIndex = 0;
    
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Function that allocates CBUFF-EDMA channel
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 */
static void MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    uint32_t chainChannel, shadowChannel;

    if(ptrEDMAInfo->dmaNum == 0)
    {
        chainChannel      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_0;
        shadowChannel     = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0;
        allocateEDMAChannel(gMmwMssMCB.edmaHandle1, &chainChannel,
                            &chainChannel,
                            &chainChannel);
        allocateEDMAShadowChannel(gMmwMssMCB.edmaHandle1, &shadowChannel);
    } 
    else
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }
    ptrEDMACfg->chainChannelsId      = chainChannel;
    ptrEDMACfg->shadowLinkChannelsId = shadowChannel;

exit:
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF Session
 *
 *  @param[in]  ptrEDMAInfo
 *      Pointer to the EDMA Information
 *  @param[out]  ptrEDMACfg
 *      Populated EDMA channel configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_LVDSStream_AllocateCBUFFChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        MmwDemo_LVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo, ptrEDMACfg);
        retVal = 0;
    }
    else
    {
        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->sessionEDMAChannelTable[streamMCBPtr->sessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));

        /* Increment the allocator index: */
        streamMCBPtr->sessionEDMAChannelAllocatorIndex++;

        /* EDMA Channel allocated successfully */
        retVal = 0;
    }    

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the registered CBUFF EDMA channel free function which frees EDMA channels
 *      which had been allocated for use by a CBUFF Session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_EDMAFreeCBUFFChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    if(ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_0)
    {
        /*This is the CBUFF trigger channel.*/
        goto exit;  
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    MmwDemo_debugAssert (0);

exit:
    return;
}

/**
 *  @b Description
 *  @n
 *      This function deletes the cbuff session 
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_LVDSStreamDeleteSession (void)
{
    int32_t     errCode;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    
    if (CBUFF_deactivateSession (streamMcb->sessionHandle, &errCode) < 0)
    {
        CLI_write ("Error: Unable to deactivate the session [Error code %d]\r\n", errCode);
    }

    /* Delete session*/
    if (CBUFF_close (streamMcb->sessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        CLI_write ("Error: MmwDemo_LVDSStreamDeleteSession CBUFF_close failed. Error code %d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    streamMcb->sessionHandle = NULL;
}

/**
 *  @b Description
 *  @n
 *      This is the registered callback function which is invoked after the
 *      frame done interrupt is received for the cbuff session.
 *
 *  @param[in]  sessionHandle
 *      Handle to the session
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_LVDSStream_TriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{
    /* Increment stats*/
    gMmwMssMCB.lvdsStream.frameDoneCount++;

    SemaphoreP_post(&gMmwMssMCB.lvdsStream.frameDoneSemHandle);
}

/**
 *  @b Description
 *  @n
 *      This is the LVDS streaming config function. 
 *      It configures the sessions for the LVDS streaming.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_LVDSStreamConfig (void)
{
    CBUFF_SessionCfg          sessionCfg;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;

    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));
    
    /* Populate the configuration: */
    sessionCfg.edmaHandle             = gMmwMssMCB.edmaHandle1;
    sessionCfg.allocateEDMAChannelFxn = MmwDemo_LVDSStream_AllocateCBUFFChannel;
    sessionCfg.freeEDMAChannelFxn     = MmwDemo_LVDSStream_EDMAFreeCBUFFChannel;
    sessionCfg.frameDoneCallbackFxn   = MmwDemo_LVDSStream_TriggerFrameDone;
    sessionCfg.dataType               = CBUFF_DataType_REAL;
    sessionCfg.executionMode          = CBUFF_SessionExecuteMode_SW;
    sessionCfg.u.swCfg.userBufferInfo[0].size     = gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * gMmwMssMCB.numRxAntennas;
    sessionCfg.u.swCfg.userBufferInfo[0].address  = CSL_DSS_ADCBUF_READ_U_BASE;
       
    /* Create the CBUFF Session: */
    streamMcb->sessionHandle = CBUFF_createSession (gMmwMssMCB.lvdsStream.cbuffHandle, &sessionCfg, &errCode);
                                                      
    if (streamMcb->sessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF session */
        CLI_write("Error: MmwDemo_LVDSStream_config unable to create the CBUFF session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*      High level API for configuring the session. Deletes the session if it exists,
*      configures desired configuration input and activates the session
*
*  @retval
*      None
*/
void MmwDemo_configLVDSData(void)
{
    int32_t retVal;

    /* Delete previous CBUFF session if one was configured */
    if(gMmwMssMCB.lvdsStream.sessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSession();
    }

    /* Configure LVDS session */
    if (MmwDemo_LVDSStreamConfig() < 0)
    {
        CLI_write("Failed LVDS stream configuration\n");
        MmwDemo_debugAssert(0);
    }

    if(CBUFF_activateSession(gMmwMssMCB.lvdsStream.sessionHandle, &retVal) < 0)
    {
        CLI_write("Failed to activate CBUFF session for LVDS stream. errCode=%d\n",retVal);
        MmwDemo_debugAssert(0);
    }
}

/**
*  @b Description
*  @n
*      High level API for triggering the sw session.
*
*  @retval
*      None
*/
void MmwDemo_configLVDSDataTrigger(void)
{
    int32_t retVal;

    if(CBUFF_triggerSWSession(gMmwMssMCB.lvdsStream.sessionHandle, &retVal) < 0)
    {
        CLI_write("Failed to activate CBUFF session for LVDS stream. errCode=%d\n",retVal);
        MmwDemo_debugAssert(0);
    }
}
