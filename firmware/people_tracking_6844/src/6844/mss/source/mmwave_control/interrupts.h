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

/*This is the ISR Handler for Monitors*/
#if (ENABLE_MONITORS==1)
void mmwDemoMonitorISR(void);
#endif

/*For debugging purposes*/
//#define ENABLE_BURST_INTERRUPT

/*For debugging purposes*/
//#define ENABLE_CHIRP_AVAILABLE_INTERRUPT

/*For debugging purposes*/
#ifdef ENABLE_BURST_INTERRUPT
/*This is to register Burst Interrupt*/
int32_t MmwDemo_registerBurstInterrupt(void);
#endif
#if 0
/*This is to register Chirpt Interrupt*/
int32_t mmwDemo_registerChirpInterrupt(void);
#endif
#ifdef ENABLE_CHIRP_AVAILABLE_INTERRUPT
/*This is to register Chirp Available Interrupt*/
extern int32_t mmwDemo_registerChirpAvailableInterrupts(void);
#endif

/*This is to register Frame Start Interrupt*/
int32_t MmwDemo_registerFrameStartInterrupt(void);

/*For debugging purposes*/
#ifdef ENABLE_BURST_INTERRUPT
/*Burst ISR*/
void MmwDemoBurstISR(void *arg);
#endif
#if 0
/*Chirp Start ISR*/
void mmwDemoChirpStartISR(void *arg);
#endif
//#ifdef ENABLE_CHIRP_AVAILABLE_INTERRUPT
/*Chirp ISR*/
static void mmwDemoChirpISR(void *arg);
//#endif

/*Frame start ISR*/
static void mmwDemoFrameStartISR(void *arg);