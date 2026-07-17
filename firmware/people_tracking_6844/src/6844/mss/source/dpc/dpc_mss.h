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

#ifndef DPC_H_
#define DPC_H_
/*DPC Object Detection functions*/
uint8_t DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool);
void DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(HwaDmaTrigChanPoolObj *pool);
int16_t DPC_ObjDet_HwaWinRamMemoryPoolAlloc(HwaWinRamMemoryPoolObj *pool, uint16_t numSamples);
void DPC_ObjDet_HwaWinRamMemoryPoolReset(HwaWinRamMemoryPoolObj *pool);
void DPC_ObjDet_MemPoolReset(MemPoolObj *pool);
void DPC_ObjDet_MemPoolSet(MemPoolObj *pool, void *addr);
void *DPC_ObjDet_MemPoolGet(MemPoolObj *pool);
uint32_t DPC_ObjDet_MemPoolGetMaxUsage(MemPoolObj *pool);
void *DPC_ObjDet_MemPoolAlloc(MemPoolObj *pool,uint32_t size,uint8_t align);


/*Utility functions*/
void MmwDemo_compressPointCloudList(MmwDemo_output_message_UARTpointCloud *pointCloudOut,
                                    MmwDemo_output_message_point_unit *pointCloudUintRecip,
                                    DPIF_PointCloudCartesianExt *pointCloudIn,
                                    uint32_t numPoints);
void rangeBiasRxChPhaseMeasure_quadfit(float *x, float*y, float *xv, float *yv);
void MmwDemo_calcActiveAntennaGeometry();


/* Running modes */
#define DPC_RUNNING_MODE_INDET 0
#define DPC_RUNNING_MODE_SBR 1
#define DPC_RUNNING_MODE_CPD 2

/*DPU init functions*/
void rangeProc_dpuInit();

/*DPU config functions*/
void mmwDemo_rangeProcConfig(uint32_t *hwaParamSetStartIdx,
                             uint32_t *hwaWinRamOffset);

/*DPU config parser functions*/
int32_t RangeProc_configParser(uint32_t *hwaParamSetStartIdx,
                               uint32_t *hwaWinRamOffset);

                            
// Tracker DPU config
void DPC_ObjDet_TrackerDpuInit();
void DPC_ObjDet_TrackerDpuCfg(void);              

/*Function initiliazing all indvidual DPUs*/
void DPC_Init();
/*Function configuring all DPUs*/
void DPC_Config();
/*DPC processing chain execute function.*/
void DPC_Execute();

void DPC_ObjectDetection_Profile(DPC_ObjectDetectionRangeHWA_ProfileTimeStamp *stamp);

#endif


