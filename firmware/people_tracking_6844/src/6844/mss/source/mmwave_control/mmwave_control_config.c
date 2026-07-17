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
#include <stdio.h>

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <source/mmwave_demo_mss.h>

extern MmwDemo_MSS_MCB gMmwMssMCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
static void MmwDemo_ADCBufConfig
(
    uint16_t rxChannelEn,
    uint32_t chanDataSize
)
{
    uint8_t channel = 0;
    uint16_t offset = 0;
    ADCBuf_RxChanConf rxChanConf;

    memset((void *)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    /* Enable Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(rxChannelEn & (0x1U << channel))
        {
            /* Enable Channel and configure offset. */
            rxChanConf.channel = channel;
            rxChanConf.offset = offset;
            ADCBuf_control(gMmwMssMCB.adcBuffHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);

            /* Calculate offset for the next channel */
            offset  += chanDataSize;
        }
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to populate all the required control configurations
 *
 *  @param[out]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_populateControlCfg ()
{

    MmwDemo_ADCBufConfig(gMmwMssMCB.cliRxEnbl, (((gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * sizeof(int16_t))+15)/16)*16);

    return;
}

