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
#ifndef MMW_DEMO_RES_H
#define MMW_DEMO_RES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/edma.h>
#include <drivers/hw_include/cslr_soc.h>

/*******************************************************************************
 * Resources for Object Detection DPC, currently the only DPC and hwa/edma
 * resource used in the demo.
 *******************************************************************************/
/*Shadow channels allocated from physical EDMA channels #24-51 since the count has been reduced on xwrL68xx*/
#define DPC_OBJDET_EDMA_SHADOW_BASE                                         EDMA_DSS_TPCC_A_EVT_SPI1_DMA_RX_REQ


/************************************************** Range DPU ************************************************/
#define DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH                                  EDMA_DSS_TPCC_A_CHIRP_AVAIL_IRQ
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1DINSIGNATURE_CH_ID                   (EDMA_DSS_TPCC_A_EVT_FREE_5)
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1DIN_SHADOW_LINK_CH_ID                (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1DINSIGNATURE_PING_SHADOW_LINK_CH_ID  (DPC_OBJDET_EDMA_SHADOW_BASE - 3)

/* 1D -ping */
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CH_ID                         EDMA_DSS_TPCC_A_EVT_DSS_HW_ACC_CHANNEL_TRIGGER_0
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CHAIN_CH_ID                   EDMA_DSS_TPCC_A_EVT_FREE_6
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_SHADOW_LINK_CH_ID             DPC_OBJDET_EDMA_SHADOW_BASE
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_ONE_HOT_SHADOW_LINK_CH_ID     (DPC_OBJDET_EDMA_SHADOW_BASE + 2)

/* 1D - pong */
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PONG_CH_ID                         EDMA_DSS_TPCC_A_EVT_DSS_HW_ACC_CHANNEL_TRIGGER_1
#define DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PONG_SHADOW_LINK_CH_ID             (DPC_OBJDET_EDMA_SHADOW_BASE + 1)


/************************************************** Doppler DPU ************************************************/
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PING                 EDMA_DSS_TPCC_A_EVT_FREE_7
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PONG                 EDMA_DSS_TPCC_A_EVT_FREE_8

/*This has to match the HWA DMA number*/
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PING                EDMA_DSS_TPCC_A_EVT_DSS_HW_ACC_CHANNEL_TRIGGER_2

/*This has to match the HWA DMA number*/
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PONG                EDMA_DSS_TPCC_A_EVT_DSS_HW_ACC_CHANNEL_TRIGGER_3

#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PING             EDMA_DSS_TPCC_A_EVT_FREE_9
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PONG             EDMA_DSS_TPCC_A_EVT_FREE_10

/*EDMA shadow channels*/        
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PING_SHADOW          (DPC_OBJDET_EDMA_SHADOW_BASE + 5U)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PONG_SHADOW          (DPC_OBJDET_EDMA_SHADOW_BASE + 6U)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PING_SHADOW         (DPC_OBJDET_EDMA_SHADOW_BASE + 7U)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PONG_SHADOW         (DPC_OBJDET_EDMA_SHADOW_BASE + 8U)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PING_SHADOW      (DPC_OBJDET_EDMA_SHADOW_BASE + 9U)
#define DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PONG_SHADOW      (DPC_OBJDET_EDMA_SHADOW_BASE + 10U)


/************************************************** CFAR DPU ************************************************/
#define DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_CH                                  EDMA_DSS_TPCC_A_EVT_FREE_11
#define DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_CH                              EDMA_DSS_TPCC_A_EVT_FREE_12

#define DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SHADOW                              EDMA_DSS_TPCC_A_EVT_FREE_13
#define DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_EVENT_QUE                           0
#define DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_SHADOW                          EDMA_DSS_TPCC_A_EVT_FREE_14
#define DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_EVENT_QUE                       0


/************************************************** AOA2D DPU ************************************************/
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_0                                   EDMA_DSS_TPCC_A_EVT_FREE_15
#define DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PING_EVENT_QUE                       0
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_0                        EDMA_DSS_TPCC_A_EVT_DSS_HW_ACC_CHANNEL_TRIGGER_4
#define DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PING_EVENT_QUE                      0
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_0                              (DPC_OBJDET_EDMA_SHADOW_BASE + 11U)                              
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_1                              (DPC_OBJDET_EDMA_SHADOW_BASE + 12U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_2                              (DPC_OBJDET_EDMA_SHADOW_BASE + 13U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_3                              (DPC_OBJDET_EDMA_SHADOW_BASE + 14U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_4                              (DPC_OBJDET_EDMA_SHADOW_BASE + 15U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_5                              (DPC_OBJDET_EDMA_SHADOW_BASE + 16U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_12                             (DPC_OBJDET_EDMA_SHADOW_BASE + 17U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_14                             (DPC_OBJDET_EDMA_SHADOW_BASE + 18U)


#define DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_1                                   EDMA_DSS_TPCC_A_EVT_FREE_16
#define DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PONG_EVENT_QUE                       0
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_1                        EDMA_DSS_TPCC_A_EVT_DSS_HW_ACC_CHANNEL_TRIGGER_5
#define DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PONG_EVENT_QUE                      0
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_6                              (DPC_OBJDET_EDMA_SHADOW_BASE + 19U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_7                              (DPC_OBJDET_EDMA_SHADOW_BASE + 20U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_8                              (DPC_OBJDET_EDMA_SHADOW_BASE + 21U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_9                              (DPC_OBJDET_EDMA_SHADOW_BASE + 22U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_10                             (DPC_OBJDET_EDMA_SHADOW_BASE + 23U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_11                             (DPC_OBJDET_EDMA_SHADOW_BASE + 24U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_13                             (DPC_OBJDET_EDMA_SHADOW_BASE + 25U)
#define DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_15                             (DPC_OBJDET_EDMA_SHADOW_BASE + 26U)

/*************************LVDS streaming EDMA resources*******************************/

/* CBUFF EDMA trigger channel */
#define MMW_LVDS_STREAM_CBUFF_EDMA_CH_0         EDMA_DSS_TPCC_A_EVT_DSS_CBUFF_DMA_REQ0

/*shadow CBUFF trigger channel*/
#define MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0   (DPC_OBJDET_EDMA_SHADOW_BASE + 27U)

#ifdef __cplusplus
}
#endif

#endif /* MMW_DEMO_RES_H */

