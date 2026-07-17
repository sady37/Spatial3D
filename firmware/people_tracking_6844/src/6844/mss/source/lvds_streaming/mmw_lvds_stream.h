/**
 *   @file  mmw_lvds_stream.h
 *
 *   @brief
 *      LVDS stream header file.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 - 2021 Texas Instruments, Inc.
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
#ifndef MSS_LVDS_STREAM_H
#define MSS_LVDS_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel/dpl/SemaphoreP.h>

/**
 * @brief   This is the maximum number of EDMA Channels which is used by
 * the CBUFF Session
 */
#define MMWDEMO_LVDS_STREAM_SESSION_MAX_EDMA_CHANNEL             7U

/**
 * @brief
 *  LVDS streaming MCB
 *
 * @details
 *  The LVDS streaming MCB.
 */
typedef struct MmwDemo_LVDSStream_MCB
{
    /**
    * @brief   Handle to the CBUFF Driver
    */
    CBUFF_Handle             cbuffHandle;

    /**
     * @brief   EDMA Channel Allocator Index for the Session
     */
    uint8_t                  sessionEDMAChannelAllocatorIndex;

    /**
     * @brief   EDMA Channel Resource Table: This is used for creating the CBUFF Session.
     */
    CBUFF_EDMAChannelCfg     sessionEDMAChannelTable[MMWDEMO_LVDS_STREAM_SESSION_MAX_EDMA_CHANNEL];
    
    /**
     * @brief   Handle to the CBUFF Session Handle.
     */
    CBUFF_SessionHandle      sessionHandle;
    
    /**
     * @brief   Number of frame done interrupt received.
     */
    uint16_t                 frameDoneCount;
    
    /**
     * @brief   Semaphore handle to signal cbuff session done.
     */
    SemaphoreP_Object         frameDoneSemHandle;

} MmwDemo_LVDSStream_MCB_t;

int32_t MmwDemo_LVDSStreamInit (void);
int32_t MmwDemo_LVDSStreamConfig (void);
void MmwDemo_configLVDSData(void);
void MmwDemo_LVDSStreamDeleteSession (void);
void MmwDemo_configLVDSDataTrigger (void);

#ifdef __cplusplus
}
#endif

#endif
