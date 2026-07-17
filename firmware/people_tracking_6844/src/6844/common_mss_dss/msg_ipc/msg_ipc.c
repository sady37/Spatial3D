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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <stdio.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <drivers/ipc_notify.h>
#include <drivers/uart.h>
#include <drivers/prcm.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/csl_complex_math_types.h>
#include <utils/mathutils/mathutils.h>
#include <common/syscommon.h>
#include <datapath/dpu/cfarproc/v1/cfarprochwa.h>
#include <drivers/gpio.h>
#include <drivers/edma.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <common_mss_dss/msg_ipc/msg_ipc.h>


void MsgIpc_Config(MsgIpc_CtrlObj * obj,
                    MsgIpc_Cfg * cfg)
{
    int32_t status;

    obj->msgChanId      = cfg->msgChanId;
    obj->remoteCoreId   = cfg->remoteCoreId;
    obj->msgCallback    = cfg->msgCallback;
    obj->arg            = cfg->arg;

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(obj->msgChanId, obj->msgCallback, obj->arg);
    DebugP_assert(status==SystemP_SUCCESS);

    ClockP_usleep(1000);
}

void MsgIpc_Sync()
{
    ClockP_usleep(1000);
    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);
}

void MsgIpc_sendMessage(MsgIpc_CtrlObj * obj, uint32_t message, uint32_t arg)
{
    int32_t status;
    uint64_t msgValue = (((0x000000000000FFFF & (uint64_t)message)) << 32)  |  (uint64_t)arg;

    /* send message's to remote core, wait for message to be put in HW FIFO */
    status = IpcNotify_sendMsg(obj->remoteCoreId, obj->msgChanId, msgValue, 1);
    DebugP_assert(status==SystemP_SUCCESS);
}
