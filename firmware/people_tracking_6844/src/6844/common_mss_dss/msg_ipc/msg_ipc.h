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

#ifndef MSG_PASS_DSS_H
#define MSG_PASS_DSS_H

#ifdef __cplusplus
extern "C" {
#endif

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <drivers/uart.h>
#include <drivers/soc.h>
#include <drivers/ipc_notify.h>
#include <common/syscommon.h>

/**************************************************************************
 **************************************************************************/

#define DPC_MSS_TO_DSS_PRE_START_CONFIG         0
#define DPC_MSS_TO_DSS_RADAR_CUBE_READY         1
#define DPC_MSS_TO_DSS_STOP                     2
#define DPC_MSS_TO_DSS_NUM_EVENTS               3


#define DPC_DSS_TO_MSS_CONFIGURATION_COMPLETED  100
#define DPC_DSS_TO_MSS_POINT_CLOUD_READY        101


/**************************************************************************
 **************************  Data Structures ***************************
 **************************************************************************/



typedef struct MsgIpc_CtrlObj_t
{
    /**
     * @brief   Message channel ID over which the message is sent
     */
    uint32_t    msgChanId;

    /**
     * @brief   Remote core ID (DSP core ID)
     */
    uint32_t    remoteCoreId;

    /**
     * @brief   Message callback function
     */
    IpcNotify_FxnCallback msgCallback;

    /**
     * @brief   Callback argument
     */
    void *arg;

/**
     * @brief   Message IPC initialized
     */
    bool    isMsgIpcInitialized;

} MsgIpc_CtrlObj;

typedef struct MsgIpc_Cfg_t
{
    /**
     * @brief   Message channel ID over which the message is sent
     */
    uint32_t    msgChanId;

    /**
     * @brief   Remote core ID (DSP core ID)
     */
    uint32_t    remoteCoreId;

    /**
     * @brief   Message callback function
     */
    IpcNotify_FxnCallback msgCallback;

    /**
     * @brief   Callback argument
     */
    void *arg;

} MsgIpc_Cfg;


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
void MsgIpc_Config(MsgIpc_CtrlObj * obj,
                    MsgIpc_Cfg * cfg);

void MsgIpc_Sync();

void MsgIpc_sendMessage(MsgIpc_CtrlObj * obj, uint32_t message, uint32_t arg);




#ifdef __cplusplus
}
#endif

#endif /* MMW_CLI_H */
