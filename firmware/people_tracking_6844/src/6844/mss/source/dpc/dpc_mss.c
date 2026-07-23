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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
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
#include <control/mmwave/mmwave.h>
#include <source/mmw_cli.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <drivers/power.h>
#include <drivers/prcm.h>
#include <drivers/hwa.h>
#include <drivers/edma.h>
#include <utils/mathutils/mathutils.h>

#include <source/mmwave_demo_mss.h>
#include <source/mmw_res.h>
#include <source/dpc/dpc_mss.h>
#include <source/mmwave_control/interrupts.h>
#include <source/calibrations/range_phase_bias_measurement.h>
#include <source/utils/mmw_demo_utils.h>
#include <common_mss_dss/msg_ipc/msg_ipc.h>
#include <source/lvds_streaming/mmw_lvds_stream.h>

#include <datapath/dpu/trackerproc/v0/trackerproc.h>
#include <datapath/dpu/rangeproc/v1/rangeprochwa.h>
#include "../tracker_utils/tracker_utils.h"

/**************************************************************************
 *************************** Macros Definitions ***************************
 **************************************************************************/

#define HWA_MAX_NUM_DMA_TRIG_CHANNELS 16
#define MAX_NUM_DETECTIONS          (MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE)

#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)

#define MMW_DEMO_MAJOR_MODE 0
#define MMW_DEMO_MINOR_MODE 1

#define FRAME_REF_TIMER_CLOCK_MHZ  40

/* Max Frame Size for FTDI chip is 64KB */
#define MAXSPISIZEFTDI               (65536U)

#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_HANNING
#define DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL   0
#define DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL   1
#define DPC_OBJDET_QFORMAT_DOPPLER_FFT 17

#define MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3e8)

#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_HANNING
#define DPC_DPU_MACRO_DOPPLERPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_HANNING

#define DPC_OBJDET_QFORMAT_DOPPLER_FFT 17
#define DPC_OBJDET_QFORMAT_MACRO_DOPPLER_FFT 17

#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET 0
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_BLACKMAN
#define DPC_OBJDET_QFORMAT_RANGE_FFT 17
#define MMW_DEMO_TEST_ADC_BUFF_SIZE 1024  //maximum 128 real samples (int16_t), 3 Rx channels

#define DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT))

#define DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT))

#define DPC_OBJDET_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCHWA_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT, \
                                                                   sizeof(float)))                                                                                                                           

#define DPC_OBJDETRANGE_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES 1024 //SOC_HWA_WINDOW_RAM_SIZE_IN_SAMPLES

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern HWA_Handle gHwaHandle;

/*! L3 RAM Buffer to store the radar cube and other processed signals*/
extern uint8_t gMmwL3[MSS_L3_MEM_SIZE]  __attribute((section(".l3")));

/*! Local RAM buffer for object detection DPC */
extern uint8_t gMmwCoreLocMem[MSS_CORE_LOCAL_MEM_SIZE];

/*! Local2 RAM buffer size */
extern uint8_t gMmwCoreLocMem2[MSS_CORE_LOCAL_MEM2_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

extern TaskHandle_t    gDspPointCloudTask;
extern StaticTask_t    gDspPointCloudTaskObj;
extern StackType_t     gDspPointCloudTaskStack[];


// LED config
extern uint32_t gGpioBaseAddrLed, gPinNumLed;
extern MMWave_temperatureStats  gTempStats;
extern int32_t MmwDemo_registerChirpAvailableInterrupts(void);

extern int32_t  MmwDemo_rangeBiasRxChPhaseMeasureConfig ();
extern void MmwDemo_rangeBiasRxChPhaseMeasure ();
extern void MmwDemo_dspPointCloudTask();

/**************************************************************************
 *************************** Local  Definitions ***************************
 **************************************************************************/
void DPC_mss_MsgHandler(uint32_t remoteCoreId, uint16_t localClientId, uint64_t msgValue, int32_t crcStatus, void *arg);
void MmwDemo_FillTrackerSensorPositionCfg();

Edma_IntrObject     intrObj_rangeProc[2];

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/*! @brief     DPU Configuraiton Objects */
DPU_RangeProcHWA_Config    rangeProcDpuCfg;
DPU_DopplerProcHWA_Config   gDopplerProcDpuCfg;
DPU_CFARProcHWA_Config      gCfarProcDpuCfg;
DPU_AoAProcHWA_Config       gAoa2dProcDpuCfg;

/*! @brief     EDMA interrupt objects for DPUs */
Edma_IntrObject             gEdmaIntrObjRng;
Edma_IntrObject             gEdmaIntrObjDoppler;

volatile unsigned long long test;

uint32_t gPeriodicCount = 0;
#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>

void DPC_SensorStartClockCallback(ClockP_Object *obj, void *arg)
{
    uint32_t *value = (uint32_t*)arg;

    (*value)++; /* increment number of time's this callback is called */


    /*  Restart Sensor - sensor frame trigger */
    T_RL_API_SENSOR_START_CMD sensStartCmd ={0};
    int32_t retVal = rl_sensSensorStart(0, &sensStartCmd);
    DebugP_assert(retVal == 0);

}


/* Total memory used by the intrusion detection library */
uint32_t gIntrDetectMemoryUsed = 0;
void *inDetect_malloc(uint32_t sizeInBytes)
{
    gIntrDetectMemoryUsed += sizeInBytes;
    return HeapP_alloc(&gMmwMssMCB.CoreLocalRtosHeapObj, sizeInBytes);
}
void inDetect_free(void *pFree, uint32_t sizeInBytes)
{
    gIntrDetectMemoryUsed -= sizeInBytes;
    HeapP_free(&gMmwMssMCB.CoreLocalRtosHeapObj, pFree);
}

/* Total memory used by the feature extraction library */
uint32_t gFeatExtractMemoryUsed = 0;
void *featExtract_malloc(uint32_t sizeInBytes)
{
    gFeatExtractMemoryUsed += sizeInBytes;
    return HeapP_alloc(&gMmwMssMCB.CoreLocalRtosHeapObj, sizeInBytes);
}
void featExtract_free(void *pFree, uint32_t sizeInBytes)
{
    gFeatExtractMemoryUsed -= sizeInBytes;
    HeapP_free(&gMmwMssMCB.CoreLocalRtosHeapObj, pFree);
}

/* Total memory used by the occupancy classifier library */
uint32_t gDspPointCloudMemoryUsed = 0;
void *classifier_malloc(uint32_t sizeInBytes)
{
    gDspPointCloudMemoryUsed += sizeInBytes;
    return HeapP_alloc(&gMmwMssMCB.CoreLocalRtosHeapObj, sizeInBytes);
}
void classifier_free(void *pFree, uint32_t sizeInBytes)
{
    gDspPointCloudMemoryUsed -= sizeInBytes;
    HeapP_free(&gMmwMssMCB.CoreLocalRtosHeapObj, pFree);
}

/* Total memory used by the cnn classifier library */
uint32_t gcnnClassifierMemoryUsed = 0;
void *cnn_classifier_malloc(uint32_t sizeInBytes)
{
    gcnnClassifierMemoryUsed += sizeInBytes;
    return HeapP_alloc(&gMmwMssMCB.CoreLocalRtosHeapObj, sizeInBytes);
}
void cnn_classifier_free(void *pFree, uint32_t sizeInBytes)
{
    gcnnClassifierMemoryUsed -= sizeInBytes;
    HeapP_free(&gMmwMssMCB.CoreLocalRtosHeapObj, pFree);
}

uint32_t coreLocalRtosHeap_memUsage()
{
    uint32_t usedMemSizeInBytes;
    HeapP_MemStats heapStats;

    HeapP_getHeapStats(&gMmwMssMCB.CoreLocalRtosHeapObj, &heapStats);
    usedMemSizeInBytes = sizeof(gMmwCoreLocMem2) - heapStats.availableHeapSpaceInBytes;

    return usedMemSizeInBytes;
}

/**
*  @b Description
*  @n
*        Function configuring and executing DPC
*/
void MmwDemo_dpcTask();

void DPC_ObjectDetection_Profile(DPC_ObjectDetectionRangeHWA_ProfileTimeStamp *stamp)
{
    stamp->timeInUsec = (Cycleprofiler_getTimeStamp() - gMmwMssMCB.stats.frameStartTimeStamp[stamp->rdInd])/ FRAME_REF_TIMER_CLOCK_MHZ;
    stamp->rdInd = (stamp->rdInd + 1) & 0x3;
}


/**
 *  @b Description
 *  @n
 *      Allocates Shawdow paramset
 */
static void DPC_ObjDet_AllocateEDMAShadowChannel(uint32_t *param)
{
    int32_t             testStatus = SystemP_SUCCESS;

    testStatus = EDMA_allocParam(gEdmaHandle[0], param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    return;
}

/**
 *  @b Description
 *  @n
 *      The function allocates HWA DMA source channel from the pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      channel Allocated HWA trigger source channel
 */
uint8_t DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool)
{
    uint8_t channel = 0xFF;
    if(pool->dmaTrigSrcNextChan < HWA_MAX_NUM_DMA_TRIG_CHANNELS)
    {
        channel = pool->dmaTrigSrcNextChan;
        pool->dmaTrigSrcNextChan++;
    }
    return channel;
}

/**
 *  @b Description
 *  @n
 *      The function resets HWA DMA source channel pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      none
 */
void DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(HwaDmaTrigChanPoolObj *pool)
{
    pool->dmaTrigSrcNextChan = 0;
}

/**
 *  @b Description
 *  @n
 *      The function allocates memory in HWA RAM memory pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      startSampleIndex sample index in the HWA RAM memory
 */
int16_t DPC_ObjDet_HwaWinRamMemoryPoolAlloc(HwaWinRamMemoryPoolObj *pool, uint16_t numSamples)
{
    int16_t startSampleIndex = -1;
    if((pool->memStartSampleIndex + numSamples) < (CSL_DSS_HWA_WINDOW_RAM_U_SIZE/sizeof(uint32_t)))
    {
        startSampleIndex = pool->memStartSampleIndex;
        pool->memStartSampleIndex += numSamples;
    }
    return startSampleIndex;
}

/**
 *  @b Description
 *  @n
 *      The function resets HWA DMA source channel pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      none
 */
void DPC_ObjDet_HwaWinRamMemoryPoolReset(HwaWinRamMemoryPoolObj *pool)
{
    pool->memStartSampleIndex = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
void DPC_ObjDet_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}


/**
 *  @b Description
 *  @n
 *      Utility function for setting memory pool to desired address in the pool.
 *      Helps to rewind for example.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  addr Address to assign to the pool's current address.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void DPC_ObjDet_MemPoolSet(MemPoolObj *pool, void *addr)
{
    pool->currAddr = (uintptr_t)addr;
    pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting memory pool current address.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to current address of the pool (from which next allocation will
 *      allocate to the desired alignment).
 */
void *DPC_ObjDet_MemPoolGet(MemPoolObj *pool)
{
    return((void *)pool->currAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting maximum memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
uint32_t DPC_ObjDet_MemPoolGetMaxUsage(MemPoolObj *pool)
{
    return((uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr));
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  size Size in bytes to be allocated.
 *  @param[in]  align Alignment in bytes
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
void *DPC_ObjDet_MemPoolAlloc(MemPoolObj *pool,
                              uint32_t size,
                              uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;

    addr = MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
    }

    return(retAddr);
}

/**
*  @b Description
*  @n
*    Select coordinates of active virtual antennas and calculate the size of the 2D virtual antenna pattern,
*    i.e. number of antenna rows and number of antenna columns.
*/
void MmwDemo_calcActiveAntennaGeometry()
{
    int32_t txInd, rxInd, ind;
    int32_t rowMax, colMax;
    int32_t rowMin, colMin;
    /* Select only active antennas */
    ind = 0;
    for (txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            gMmwMssMCB.activeAntennaGeometryCfg.ant[ind] = gMmwMssMCB.antennaGeometryCfg.ant[gMmwMssMCB.rxAntOrder[rxInd] + (txInd * SYS_COMMON_NUM_RX_CHANNEL)];
            ind++;
        }
    }

    /* Calculate virtual antenna 2D array size */
    ind = 0;
    rowMax = 0;
    colMax = 0;
    rowMin = 127;
    colMin = 127;
    for (txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row > rowMax)
            {
                rowMax = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row;
            }
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col > colMax)
            {
                colMax = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
            }
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row < rowMin)
            {
                rowMin = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row;
            }
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col < colMin)
            {
                colMin = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
            }
            ind++;
        }
    }
    ind = 0;
    for (txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row -= rowMin;
            gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col -= colMin;
            ind++;
        }
    }
    gMmwMssMCB.numAntRow = rowMax - rowMin + 1;
    gMmwMssMCB.numAntCol = colMax - colMin + 1;
}

#define DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL   0
#define DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL   1

/**
*  @b Description
*  @n
*    Based on the activeAntennaGeometryCfg configures the table which used to configure
*    Doppler FFT HWA param sets in DoA DPU. THese param sets perform Doppler FFT and
*    at the same time mapping of input antennas into 2D row-column antenna array where columns
*    are in  azimuth dimension, and rows in elevation dimension.
*    It also calculates the size of 2D antenna array, ie. number of rows and number of columns.
*/
// int32_t DPC_ObjDet_cfgDopplerParamMapping(DPU_Doa3dProc_HWA_Option_Cfg *dopplerParamCfg,
//                                           MmwDemo_antennaGeometryCfg *activeAntennaGeometryCfg,
//                                           uint32_t mappingOption,
//                                           uint16_t numAntRow,
//                                           uint16_t numAntCol,
//                                           uint32_t numDopplerBins,
//                                           uint32_t numTxAntennas,
//                                           uint32_t numRxAntennas)
// {
//     int32_t ind, indNext, indNextPrev;
//     int32_t row, col;
//     int32_t dopParamInd;
//     int32_t state;
//     int16_t BT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
//     int16_t DT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
//     int16_t SCAL[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
//     int8_t  DONE[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
//     int32_t retVal = 0;
//     int32_t rowOffset;

//     if (numAntRow * numAntCol > DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS)
//     {
//         retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
//         goto exit;
//     }

//     if (mappingOption == DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL)
//     {
//         /*For AOA DPU, Output is */
//         rowOffset =  numAntCol;
//     }
//     else if (mappingOption == DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL)
//     {
//         rowOffset =  numDopplerBins * numAntCol;
//     }
//     else
//     {
//         retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
//         goto exit;
//     }

//     /* Initialize tables */
//     for (ind = 0; ind < (numAntRow * numAntCol); ind++)
//     {
//         BT[ind] = 0;
//         SCAL[ind] = 0;
//         DONE[ind] = 0;
//     }

//     for (ind = 0; ind < (numTxAntennas * numRxAntennas); ind++)
//     {
//         row = activeAntennaGeometryCfg->ant[ind].row;
//         col = activeAntennaGeometryCfg->ant[ind].col;
//         BT[row * numAntCol + col] = ind;
//         SCAL[row * numAntCol + col] = 1;
//     }
//     for (row = 0; row < numAntRow; row++)
//     {
//         for (col = 0; col < numAntCol; col++)
//         {
//             ind = row * numAntCol + col;
//             DT[ind] = row * rowOffset + col;
//         }
//     }


//     /* Configure Doppler HWA mapping params for antenna mapping */
//     dopParamInd = 0;
//     dopplerParamCfg->numDopFftParams = 0;
//     for (ind = 0; ind < (numAntRow * numAntCol); ind++)
//     {
//         if (!DONE[ind])
//         {
//             if(dopParamInd == DPU_DOA3DPROC_MAX_NUM_DOP_FFFT_PARAMS)
//             {
//                 retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
//                 goto exit;
//             }

//             DONE[ind] = 1;
//             dopplerParamCfg->numDopFftParams++;
//             dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt = 1;
//             dopplerParamCfg->dopFftCfg[dopParamInd].scale = SCAL[ind];
//             if (dopplerParamCfg->dopFftCfg[dopParamInd].scale == 0)
//             {
//                 dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = 0;
//             }
//             else
//             {
//                 dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = BT[ind];
//             }
//             dopplerParamCfg->dopFftCfg[dopParamInd].dstAddrOffset = DT[ind];
//             state = 1;//STATE_SECOND:
//             for (indNext = ind+1; indNext < (numAntRow * numAntCol); indNext++)
//             {

//                 if (!DONE[indNext] && (dopplerParamCfg->dopFftCfg[dopParamInd].scale == SCAL[indNext]))
//                 {
//                     switch (state)
//                     {
//                         case 1://STATE_SECOND:
//                             dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
//                             DONE[indNext] = 1;
//                             if (SCAL[indNext] == 1)
//                             {
//                                 dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = BT[indNext] - dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset;
//                             }
//                             else
//                             {
//                                 dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = 0;
//                             }
//                             dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx = DT[indNext] - DT[ind];
//                             indNextPrev = indNext;
//                             state = 2;//STATE_NEXT:
//                             break;
//                         case 2://STATE_NEXT:
//                             if (SCAL[indNext] == 1)
//                             {
//                                 if ((dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx == (BT[indNext] - BT[indNextPrev])) &&
//                                     (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev])))
//                                 {
//                                     DONE[indNext] = 1;
//                                     dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
//                                     indNextPrev = indNext;
//                                 }
//                             }
//                             else
//                             {
//                                 if (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev]))
//                                 {
//                                     DONE[indNext] = 1;
//                                     dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
//                                     indNextPrev = indNext;
//                                 }
//                             }
//                             break;
//                     }
//                 }
//             }
//             dopParamInd++;
//         }
//     }

//     dopplerParamCfg->numDopFftParams = dopParamInd;

// exit:
//     return retVal;
// }

/*  @b Description
*  @n
*    Range processing DPU Initialization
*/
void DPC_ObjDet_RngDpuInit()
{
    int32_t errorCode = 0;
    DPU_RangeProcHWA_InitParams initParams;
    initParams.hwaHandle = gHwaHandle;

    /* generate the dpu handler*/
    gMmwMssMCB.rangeProcDpuHandle = DPU_RangeProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.rangeProcDpuHandle == NULL)
    {
        CLI_write("Error: RangeProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert(0);
        return;
    }
}

/*  @b Description
*  @n
*    Doppler processing DPU Initialization
*/
void DPC_ObjDet_DopplerDpuInit()
{
    int32_t  errorCode = 0;
    DPU_DopplerProcHWA_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.dopplerProcDpuHandle =  DPU_DopplerProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.dopplerProcDpuHandle == NULL)
    {
        CLI_write ("Error: Doppler Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*    CFAR DPU Initialization
*/
void DPC_ObjDet_CfarDpuInit()
{
    int32_t  errorCode = 0;
    DPU_CFARProcHWA_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.cfarProcDpuHandle =  DPU_CFARProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.cfarProcDpuHandle == NULL)
    {
        CLI_write ("Error: CFAR Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*    AOA2D DPU Initialization
*/
void DPC_ObjDet_AoaDpuInit()
{
    int32_t  errorCode = 0;
    DPU_AoAProcHWA_InitParams initParams;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.aoa2dProcDpuHandle =  DPU_AoAProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.aoa2dProcDpuHandle == NULL)
    {
        CLI_write ("Error: AOA2D Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

void DPC_ObjDet_TrackerDpuInit()
{
    int32_t errorCode = 0;
    /* generate the dpu handler*/
    gMmwMssMCB.trackerProcDpuHandle = DPU_TrackerProc_init(&errorCode);
    if (gMmwMssMCB.trackerProcDpuHandle == NULL)
    {
        CLI_write("Error: Tracker Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert(0);
        return;
    }
}

int32_t DPC_ObjDet_RngDpuCfg_Parser()
{
    int32_t retVal = 0;
    DPU_RangeProcHWA_HW_Resources  *pHwConfig;
    DPU_RangeProcHWA_StaticConfig  *params;
    uint32_t index;
    uint32_t bytesPerRxChan;
    uint16_t numBytesPerInputSample;
    uint32_t dmaCh, tcc, param;

    memset((void *)&rangeProcDpuCfg, 0, sizeof(DPU_RangeProcHWA_Config));

    pHwConfig = &rangeProcDpuCfg.hwRes;
    params = &rangeProcDpuCfg.staticCfg;

    /****************** Static configurations ******************/
    params->numTxAntennas = gMmwMssMCB.numTxAntennas;
    params->numVirtualAntennas = gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas;
    params->numChirpsPerFrame = gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst;
    params->isChirpDataReal = 1; /*This device supports only real ADC data*/
    
    numBytesPerInputSample = sizeof(int16_t);
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->numFFTBins = mathUtils_pow2roundup(gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples);
    
    /* windowing */
    params->windowSize = sizeof(uint32_t) * ((gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples + 1) / 2); 
    params->window =  (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                         params->windowSize,
                                                         sizeof(uint32_t));
    if (params->window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;
        goto exit;
    }
 
    params->ADCBufData.dataProperty.numAdcSamples = gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples;
    params->ADCBufData.dataProperty.numRxAntennas = gMmwMssMCB.numRxAntennas;

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)params->window,
                            (uint32_t) params->ADCBufData.dataProperty.numAdcSamples,
                            params->windowSize/sizeof(uint32_t),
                            DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_RANGE_FFT);
    }

    params->rangeFFTtuning.fftOutputDivShift = 2;
    params->rangeFFTtuning.numLastButterflyStagesToScale = 0; /* no scaling needed as ADC is 16-bit and we have 8 bits to grow */

    bytesPerRxChan = gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples * numBytesPerInputSample;
    bytesPerRxChan = (bytesPerRxChan + 15) / 16 * 16;

    for (index = 0; index < SYS_COMMON_NUM_RX_CHANNEL; index++)
    {
        params->ADCBufData.dataProperty.rxChanOffset[index] = index * bytesPerRxChan;
    }

    params->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
    
    /* adc buffer buffer, format fixed, interleave, size will change */
    params->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_REAL16;

    params->ADCBufData.dataProperty.adcBits = 2U;
    params->ADCBufData.dataProperty.numChirpsPerChirpEvent = 1U;

    if(gMmwMssMCB.adcDataSourceCfg.source == 0)
    {
        params->ADCBufData.data = (void *)CSL_DSS_ADCBUF_READ_U_BASE;
    }
    else
    {
        gMmwMssMCB.adcTestBuff  = (uint8_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                            MMW_DEMO_TEST_ADC_BUFF_SIZE,
                                                                            sizeof(uint32_t));
        if(gMmwMssMCB.adcTestBuff == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_ADC_TEST_BUFF;
            goto exit;
        }
        params->ADCBufData.data = (void *)gMmwMssMCB.adcTestBuff;

    }

    /****************** Dynamic or HW resource configurations ******************/
    pHwConfig->intrObj = &gEdmaIntrObjRng;
    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;
    pHwConfig->hwaCfg.numParamSet = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    pHwConfig->hwaCfg.hwaWinRamOffset  = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                               mathUtils_pow2roundup(gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples)/2);
    pHwConfig->hwaCfg.hwaWinSym = 1;
    pHwConfig->hwaCfg.dataInputMode = DPU_RangeProcHWA_InputMode_ISOLATED;

    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];

    /* Data Input EDMA */
    dmaCh = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    tcc   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    param = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaInCfg.dataIn.channel         = dmaCh;
    pHwConfig->edmaInCfg.dataIn.paramId         = param;
    pHwConfig->edmaInCfg.dataIn.tcc             = tcc;

    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1DIN_SHADOW_LINK_CH_ID;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaInCfg.dataIn.shadowPramId  = param;
    pHwConfig->edmaInCfg.dataIn.eventQueue      = 0;

    dmaCh = DPC_OBJDET_DPU_RANGEPROC_EDMA_1DINSIGNATURE_CH_ID;
    tcc   = DPC_OBJDET_DPU_RANGEPROC_EDMA_1DINSIGNATURE_CH_ID;
    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1DINSIGNATURE_CH_ID;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaInCfg.dataInSignature.channel         = dmaCh;
    pHwConfig->edmaInCfg.dataInSignature.paramId         = param;
    pHwConfig->edmaInCfg.dataInSignature.tcc             = tcc;

    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1DINSIGNATURE_PING_SHADOW_LINK_CH_ID;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaInCfg.dataInSignature.shadowPramId   = param;
    pHwConfig->edmaInCfg.dataInSignature.eventQueue      = 0;

    /* Output Ping*/
    dmaCh = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CH_ID;
    tcc   = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CH_ID;
    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CH_ID;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPing.channel         = dmaCh;
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPing.paramId         = param;
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPing.tcc             = tcc;

    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_SHADOW_LINK_CH_ID;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPing.shadowPramId   = param;
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPing.eventQueue= 0;

    /* Output Pong*/
    dmaCh = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PONG_CH_ID;
    tcc   = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PONG_CH_ID;
    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PONG_CH_ID;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPong.channel         = dmaCh;
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPong.paramId         = param;
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPong.tcc             = tcc;

    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PONG_SHADOW_LINK_CH_ID;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPong.shadowPramId   = param;
    pHwConfig->edmaOutCfg.u.fmt2.dataOutPong.eventQueue       = 0;
        
    /* Output signature channel */
    dmaCh = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CHAIN_CH_ID;
    tcc   = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CHAIN_CH_ID;
    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_CHAIN_CH_ID;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaOutCfg.dataOutSignature.channel  = dmaCh;
    pHwConfig->edmaOutCfg.dataOutSignature.paramId  = param;
    pHwConfig->edmaOutCfg.dataOutSignature.tcc      = tcc;

    param = DPC_OBJDET_DPU_RANGEPROC_EDMA_1D_PING_ONE_HOT_SHADOW_LINK_CH_ID;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaOutCfg.dataOutSignature.shadowPramId = param;
    pHwConfig->edmaOutCfg.dataOutSignature.eventQueue = 0;

    /* radar cube */
    pHwConfig->radarCube.dataSize = params->numRangeBins * gMmwMssMCB.numRxAntennas * sizeof(uint32_t) * params->numChirpsPerFrame;
    pHwConfig->radarCube.datafmt = DPIF_RADARCUBE_FORMAT_2; /*Fmt7 for increased chirps processing is not supported by DPC and is currently only a DPU level demonstration*/
    gMmwMssMCB.radarCube[0].dataSize = pHwConfig->radarCube.dataSize;
    gMmwMssMCB.radarCube[0].datafmt = pHwConfig->radarCube.datafmt;

    gMmwMssMCB.radarCube[0].data  = (cmplx16ImRe_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                               gMmwMssMCB.radarCube[0].dataSize,
                                                                               sizeof(uint32_t));
    if(gMmwMssMCB.radarCube[0].data == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
        goto exit;
    }

    pHwConfig->radarCube.data  = (cmplx16ImRe_t *) gMmwMssMCB.radarCube[0].data;

    gMmwMssMCB.numDopplerChirps = (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst)/params->numTxAntennas;
    gMmwMssMCB.radarCube[1].dataSize = pHwConfig->radarCube.dataSize / gMmwMssMCB.numDopplerChirps * gMmwMssMCB.numFramesPerMinorMode;
    gMmwMssMCB.radarCube[1].datafmt = pHwConfig->radarCube.datafmt;

    gMmwMssMCB.radarCube[1].data  = (cmplx16ImRe_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                               gMmwMssMCB.radarCube[1].dataSize,
                                                                               sizeof(uint32_t));
    if(gMmwMssMCB.radarCube[1].data == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
        goto exit;
    }

exit:
    return retVal;
}

int32_t DPC_ObjDet_DopplerDpuCfg_Parser()
{   
    uint32_t retVal = 0;
    DPU_DopplerProcHWA_HW_Resources  *pHwConfig;
    DPU_DopplerProcHWA_StaticConfig  *params;
    uint32_t dmaCh, tcc, param;

    memset((void *)&gDopplerProcDpuCfg, 0, sizeof(DPU_DopplerProcHWA_Config));
    pHwConfig = &gDopplerProcDpuCfg.hwRes; 
    params = &gDopplerProcDpuCfg.staticCfg;
    
    /* Static configurations */
    params->numTxAntennas = gMmwMssMCB.numTxAntennas;
    params->numRxAntennas = gMmwMssMCB.numRxAntennas;
    params->numVirtualAntennas = gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas;
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->numDopplerChirps = (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst)/params->numTxAntennas;
    gMmwMssMCB.numDopplerChirps = params->numDopplerChirps;
    params->numDopplerBins = mathUtils_pow2roundup(params->numDopplerChirps);
    params->numPingPongPath = 2; /*Option1 for increased chirps processing is not supported by DPC and is currently only a DPU level demonstration*/
    params->isDetMatrixLogScale = 1; /*Currently DPC supports only log scale detection matrix*/
    params->log2NumDopplerBins = mathUtils_ceilLog2(params->numDopplerBins);

    /* clutter removal is implemented by zeroing the first Doppler bin */
    if(gMmwMssMCB.staticClutterRemovalEnable)
    {
        params->isStaticClutterRemovalEnabled = 1;
        gMmwMssMCB.numDopplerBins = params->numDopplerBins - 1;
    }
    else 
    {
        params->isStaticClutterRemovalEnabled = 0;
        gMmwMssMCB.numDopplerBins = params->numDopplerBins;
    }

    /* Dynamic or HW resources configurations */
    /* windowing */
    pHwConfig->hwaCfg.winSym = HWA_FFT_WINDOW_SYMMETRIC;
    pHwConfig->hwaCfg.winRamOffset = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                           params->numDopplerChirps/2);

    pHwConfig->hwaCfg.firstStageScaling = DPU_DOPPLERPROCHWA_FIRST_SCALING_DISABLED;

    if (pHwConfig->hwaCfg.winSym == HWA_FFT_WINDOW_NONSYMMETRIC)
    {
        pHwConfig->hwaCfg.windowSize = params->numDopplerChirps * sizeof(int32_t);
    }
    else
    {
        pHwConfig->hwaCfg.windowSize = ((params->numDopplerChirps + 1) / 2) * sizeof(int32_t);
    }

    pHwConfig->hwaCfg.window = (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                         pHwConfig->hwaCfg.windowSize,
                                                         sizeof(uint32_t));

    if (pHwConfig->hwaCfg.window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOPPLER_HWA_WINDOW;
        goto exit;
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)pHwConfig->hwaCfg.window,
                            (uint32_t) params->numDopplerChirps,
                            pHwConfig->hwaCfg.windowSize/sizeof(uint32_t),
                            DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_DOPPLER_FFT);
    }

    pHwConfig->radarCube.datafmt = DPIF_RADARCUBE_FORMAT_2;

    if (pHwConfig->radarCube.datafmt == DPIF_RADARCUBE_FORMAT_2)
    {
        pHwConfig->hwaCfg.numParamSets =
                DPU_DOPPLERPROCHWA_NUM_HWA_PARAMS_FMT2(params->numPingPongPath);
    }
    else
    {
        pHwConfig->hwaCfg.numParamSets =
                DPU_DOPPLERPROCHWA_NUM_HWA_PARAMS_FMT7(params->numPingPongPath, params->numTxAntennas);
    }

    pHwConfig->edmaCfg.edmaHandle = gEdmaHandle[0];
    pHwConfig->edmaCfg.intrObj = &gEdmaIntrObjDoppler;

    dmaCh = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PING;
    tcc   = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PING;
    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PING;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaCfg.edmaIn.pingPong[0].channel = dmaCh;
    pHwConfig->edmaCfg.edmaIn.pingPong[0].paramId = param;
    pHwConfig->edmaCfg.edmaIn.pingPong[0].tcc     = tcc;

    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PING_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaCfg.edmaIn.pingPong[0].shadowPramId = param;
    pHwConfig->edmaCfg.edmaIn.pingPong[0].eventQueue = 0;


    dmaCh = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PONG;
    tcc   = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PONG;
    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PONG;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaCfg.edmaIn.pingPong[1].channel = dmaCh;
    pHwConfig->edmaCfg.edmaIn.pingPong[1].paramId = param;
    pHwConfig->edmaCfg.edmaIn.pingPong[1].tcc     = tcc;

    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_IN_PONG_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaCfg.edmaIn.pingPong[1].shadowPramId = param;
    pHwConfig->edmaCfg.edmaIn.pingPong[1].eventQueue = 0;

    dmaCh = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PING;
    tcc   = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PING;
    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PING;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaCfg.edmaOut.pingPong[0].channel = dmaCh;
    pHwConfig->edmaCfg.edmaOut.pingPong[0].paramId = param;
    pHwConfig->edmaCfg.edmaOut.pingPong[0].tcc     = tcc;

    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PING_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaCfg.edmaOut.pingPong[0].shadowPramId = param;
    pHwConfig->edmaCfg.edmaOut.pingPong[0].eventQueue = 0;

    dmaCh = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PONG;
    tcc   = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PONG;
    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PONG;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaCfg.edmaOut.pingPong[1].channel = dmaCh;
    pHwConfig->edmaCfg.edmaOut.pingPong[1].paramId = param;
    pHwConfig->edmaCfg.edmaOut.pingPong[1].tcc     = tcc;

    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_OUT_PONG_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaCfg.edmaOut.pingPong[1].shadowPramId = param;
    pHwConfig->edmaCfg.edmaOut.pingPong[1].eventQueue = 0;

    dmaCh = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PING;
    tcc   = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PING;
    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PING;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaCfg.edmaHotSig.pingPong[0].channel = dmaCh;
    pHwConfig->edmaCfg.edmaHotSig.pingPong[0].paramId = param;
    pHwConfig->edmaCfg.edmaHotSig.pingPong[0].tcc     = tcc;

    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PING_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaCfg.edmaHotSig.pingPong[0].shadowPramId = param;
    pHwConfig->edmaCfg.edmaHotSig.pingPong[0].eventQueue = 0;

    dmaCh = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PONG;
    tcc   = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PONG;
    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PONG;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[0], &dmaCh, &tcc, &param);
    pHwConfig->edmaCfg.edmaHotSig.pingPong[1].channel = dmaCh;
    pHwConfig->edmaCfg.edmaHotSig.pingPong[1].paramId = param;
    pHwConfig->edmaCfg.edmaHotSig.pingPong[1].tcc     = tcc;

    param = DPC_OBJDET_DPU_DOPPLERPROC_EDMA_DOPPLERPROC_HOTSIG_PONG_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaCfg.edmaHotSig.pingPong[1].shadowPramId = param;
    pHwConfig->edmaCfg.edmaHotSig.pingPong[1].eventQueue = 0;
   
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;

    /* cube input*/
    pHwConfig->radarCube.dataSize = gMmwMssMCB.radarCube[0].dataSize;
    pHwConfig->radarCube.data = (cmplx16ImRe_t *)gMmwMssMCB.radarCube[0].data;

    /* output */
    pHwConfig->detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;
    pHwConfig->detMatrix.dataSize = params->numRangeBins * gMmwMssMCB.numDopplerBins * sizeof(uint16_t);
    
    if (params->isDetMatrixLogScale == 1)
    {
        gMmwMssMCB.detMatrix = (uint16_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                            pHwConfig->detMatrix.dataSize,
                                                            sizeof(uint32_t));
    }
    else
    {
        gMmwMssMCB.detMatrix = NULL;
    }

    if (gMmwMssMCB.detMatrix == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
        goto exit;
    }

    pHwConfig->detMatrix.data = (uint16_t *)gMmwMssMCB.detMatrix;

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the CFAR detection processing DPU configurations
*/
int32_t DPC_ObjDet_CfarDpuCfg_Parser()
{
    int32_t retVal = 0;
    float adcStart, startFreq, slope, bandwidth;
    DPU_CFARProcHWA_HW_Resources *pHwConfig;
    DPU_CFARProcHWA_StaticConfig  *params;
    DPU_CFARProcHWA_DynamicConfig *dynCfg;
    uint32_t dmaCh, tcc, param;

    memset((void *)&gCfarProcDpuCfg, 0, sizeof(DPU_CFARProcHWA_Config));
    
    /* CFARproc DPU based on Range/Doppler heatmap */
    pHwConfig = &gCfarProcDpuCfg.res;
    params = &gCfarProcDpuCfg.staticCfg;
    dynCfg = &gCfarProcDpuCfg.dynCfg;

    /* Static configurations */
    params->numDopplerBins = mathUtils_pow2roundup((gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst)/gMmwMssMCB.numTxAntennas);
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->log2NumDopplerBins = mathUtils_floorLog2(params->numDopplerBins);
    
    gMmwMssMCB.adcStartTime         =   (gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpAdcStartTime >> 10) * (1/gMmwMssMCB.adcSamplingRate); //us
    adcStart                        =   (gMmwMssMCB.adcStartTime * 1.e-6);
    startFreq                       =   (float)(gMmwMssMCB.mmWaveCfg.profileTimeCfg.startFreqGHz * 1.e9);
    slope                           =   (float)(gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpSlope * 1.e12);
    bandwidth                       =   (slope * gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples)/(gMmwMssMCB.adcSamplingRate * 1.e6);
    gMmwMssMCB.centerFreq           =   startFreq + bandwidth * 0.5f + adcStart * slope;

    params->rangeStep               =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * (gMmwMssMCB.adcSamplingRate * 1.e6)) /
                                        (2.f * slope * (2*params->numRangeBins));

    if (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame > 1)
    {
        /* Burst mode: Assumes h_NumOfBurstsInFrame > 1, h_NumOfChirpsInBurst = numTx. 
         * Below calculation may not be accurate for other combinations of h_NumOfChirpsInBurst 
         * and may need more robust technique to estimate doppler step, based on the use case */
        params->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * params->numDopplerBins *
                                            gMmwMssMCB.centerFreq * (gMmwMssMCB.mmWaveCfg.frameCfg.burstPeriodus * 1e-6));
    }
    else
    {
        /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= 2 */
        params->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst *
                                            gMmwMssMCB.centerFreq * ((gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpIdleTimeus + gMmwMssMCB.mmWaveCfg.profileComCfg.chirpRampEndTimeus) * 1e-6));
        if(gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsAccum != 0)
        {
            /* When numOfChirpsAccum is greater than zero, the chirping window will increase acccording to numOfChirpsAccum selected. */ 
            params->dopplerStep = params->dopplerStep/gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsAccum;
        }
    }

    /* clutter removal is implemented by zeroing the first Doppler bin */
    if(gMmwMssMCB.staticClutterRemovalEnable)
    {
        params->numDopplerBins = params->numDopplerBins - 1;
    }
    
    dynCfg->cfarCfgDoppler = &gMmwMssMCB.cfarDopplerCfg;
    dynCfg->cfarCfgRange = &gMmwMssMCB.cfarRangeCfg;
    dynCfg->fovRange = &gMmwMssMCB.fovRange;
    dynCfg->fovDoppler = &gMmwMssMCB.fovDoppler;
    
    /* Dynamic or HW resources configurations */
    pHwConfig->edmaHandle = gEdmaHandle[CONFIG_EDMA0]; //edmaHandle;

    dmaCh = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_CH;
    tcc   = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_CH;
    param = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_CH;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[CONFIG_EDMA0], &dmaCh, &tcc, &param);
    pHwConfig->edmaHwaIn.channel = dmaCh;
    pHwConfig->edmaHwaIn.paramId = param;
    pHwConfig->edmaHwaIn.tcc     = tcc;

    param = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaHwaIn.shadowPramId = param;
    pHwConfig->edmaHwaIn.eventQueue = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_EVENT_QUE;

    dmaCh = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_CH;
    tcc   = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_CH;
    param = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_CH;
    DPEDMA_allocateEDMAChannel(gEdmaHandle[CONFIG_EDMA0], &dmaCh, &tcc, &param);
    pHwConfig->edmaHwaInSignature.channel = dmaCh;
    pHwConfig->edmaHwaInSignature.paramId = param;
    pHwConfig->edmaHwaInSignature.tcc     = tcc;

    param = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_SHADOW;
    DPC_ObjDet_AllocateEDMAShadowChannel(&param);
    pHwConfig->edmaHwaInSignature.shadowPramId = param;
    pHwConfig->edmaHwaInSignature.eventQueue = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_EVENT_QUE;

    pHwConfig->hwaCfg.numParamSet = DPU_CFARPROCHWA_NUM_HWA_PARAMS;
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets; 

    pHwConfig->detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;
    pHwConfig->detMatrix.dataSize = params->numDopplerBins * params->numRangeBins * sizeof(uint16_t); 
    pHwConfig->detMatrix.data = (uint16_t *)gMmwMssMCB.detMatrix;

    /* Give M0 and M1 memory banks for detection matrix scratch. */
    pHwConfig->hwaMemInp = (uint16_t *) CSL_DSS_HWA_DMA0_RAM_BANK0_BASE;
    pHwConfig->hwaMemInpSize = (CSL_DSS_HWA_BANK_SIZE * 2) / sizeof(uint16_t);

    /* Entire M2 bank for doppler output */
    pHwConfig->hwaMemOutDoppler = (DPU_CFARProcHWA_CfarDetOutput *)CSL_DSS_HWA_DMA0_RAM_BANK2_BASE;
    pHwConfig->hwaMemOutDopplerSize = CSL_DSS_HWA_BANK_SIZE / sizeof(DPU_CFARProcHWA_CfarDetOutput);

    /* Entire M3 bank for range output */
    pHwConfig->hwaMemOutRange = (DPU_CFARProcHWA_CfarDetOutput *)CSL_DSS_HWA_DMA0_RAM_BANK3_BASE;
    pHwConfig->hwaMemOutRangeSize = CSL_DSS_HWA_BANK_SIZE / sizeof(DPU_CFARProcHWA_CfarDetOutput);
    
    pHwConfig->cfarDopplerDetOutBitMaskSize = (params->numRangeBins * params->numDopplerBins) / 32;
    /* Avoid cfarDopplerDetOutBitMaskSize to round down if (numRangeBins * numDopplerBins) is not a multiple of 32 */
    if(0 != ((params->numRangeBins * params->numDopplerBins) % 32))
    {
    	pHwConfig->cfarDopplerDetOutBitMaskSize += 1U;
    }

    pHwConfig->cfarDopplerDetOutBitMask = (uint32_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                                                pHwConfig->cfarDopplerDetOutBitMaskSize * sizeof(uint32_t),
                                                                                sizeof(uint32_t));

    if (pHwConfig->cfarDopplerDetOutBitMask == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_CFAR_DOPPLER_DET_OUT_BIT_MASK;
        goto exit;
    }
    
    pHwConfig->cfarRngDopSnrListSize = MAX_NUM_DETECTIONS;
    gMmwMssMCB.cfarRngDopSnrList = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                        pHwConfig->cfarRngDopSnrListSize * sizeof(DPIF_CFARDetList),
                                                        sizeof(int16_t));
    
    pHwConfig->cfarRngDopSnrList = gMmwMssMCB.cfarRngDopSnrList;
    if (pHwConfig->cfarRngDopSnrList == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_CFAR_OUT_DET_LIST;
        goto exit;
    }

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the aoa2d processing DPU configurations
*/
int32_t DPC_ObjDet_AoaDpuCfg_Parser()
{
    int32_t retVal = 0;
    DPU_AoAProcHWA_StaticConfig  *params;
    DPU_AoAProcHWA_HW_Resources  *pHwConfig;
    DPU_AoAProc_DynamicConfig   *dynCfg;
    uint8_t txInd, rxInd, ind;
    float slope;

    memset((void *)&gAoa2dProcDpuCfg, 0, sizeof(DPU_AoAProcHWA_Config));

    pHwConfig = &gAoa2dProcDpuCfg.res;
    params = &gAoa2dProcDpuCfg.staticCfg;
    dynCfg = &gAoa2dProcDpuCfg.dynCfg;

    /* Static configurations */
    params->numTxAntennas      = gMmwMssMCB.numTxAntennas;
    params->numRxAntennas      = gMmwMssMCB.numRxAntennas;
    params->numDopplerChirps   = (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst)/params->numTxAntennas;
    params->numDopplerBins     = mathUtils_pow2roundup(params->numDopplerChirps);
    params->numRangeBins       = gMmwMssMCB.numRangeBins;

    slope                      =   (float)(gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpSlope * 1.e12);

    params->rangeStep          = (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * (gMmwMssMCB.adcSamplingRate * 1.e6)) /
                                    (2.f * slope * (2*params->numRangeBins));

    if (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame > 1)
    {
        /* Burst mode: Assumes h_NumOfBurstsInFrame > 1, h_NumOfChirpsInBurst = numTx. 
         * Below calculation may not be accurate for other combinations of h_NumOfChirpsInBurst 
         * and may need more robust technique to estimate doppler step, based on the use case */
        params->dopplerStep    =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                    (2.f * params->numDopplerBins *
                                    gMmwMssMCB.centerFreq * (gMmwMssMCB.mmWaveCfg.frameCfg.burstPeriodus * 1e-6));
    }
    else
    {
        /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= 2 */
        params->dopplerStep    =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                    (2.f * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst *
                                    gMmwMssMCB.centerFreq * ((gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpIdleTimeus + gMmwMssMCB.mmWaveCfg.profileComCfg.chirpRampEndTimeus) * 1e-6));
    }

    if(gMmwMssMCB.staticClutterRemovalEnable)
    {
        params->isStaticClutterRemovalEnabled = 1;
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts == 0.)
    {
        params->lambdaOverDistX = 2.0;
    }
    else
    {
        params->lambdaOverDistX = 3e8 / (gMmwMssMCB.centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts);
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts == 0.)
    {
        params->lambdaOverDistZ = 2.0;
    }
    else
    {
        params->lambdaOverDistZ = 3e8 / (gMmwMssMCB.centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts);
    }

    params->numVirtualAnt       = params->numTxAntennas * params->numRxAntennas;

    ind = 0;
    for (txInd = 0; txInd < params->numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < params->numRxAntennas; rxInd++)
        {
            params->antForwardMapLUT[ind].rowIdx = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row; 
            params->antForwardMapLUT[ind].colIdx = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
            ind++;
        }
    }
    
    params->numAntRow = gMmwMssMCB.numAntRow;
    params->numAntCol = gMmwMssMCB.numAntCol;

    params->azimuthFftSize = gMmwMssMCB.aoaProcCfg.azimuthFftSize;
    params->elevationFftSize = gMmwMssMCB.aoaProcCfg.elevationFftSize;
    
    if(gMmwMssMCB.mmWaveCfg.profileComCfg.chirpTxMimoPatSel == 4)
    {
        params->isBpmEnabled = 1;
    }
    
    /* Dynamic or HW resources configurations */
    /* hwaCfg */
    pHwConfig->hwaCfg.numParamSet = DPU_AOAPROCHWA_NUM_HWA_PARAMS;
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;

    pHwConfig->hwaCfg.winSym = HWA_FFT_WINDOW_SYMMETRIC;
    pHwConfig->hwaCfg.winRamOffset = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                           params->numDopplerChirps/2);

    if (pHwConfig->hwaCfg.winSym == HWA_FFT_WINDOW_NONSYMMETRIC)
    {
        pHwConfig->hwaCfg.windowSize = params->numDopplerChirps * sizeof(int32_t);
    }
    else
    {
        pHwConfig->hwaCfg.windowSize = ((params->numDopplerChirps + 1) / 2) * sizeof(int32_t);
    }

    pHwConfig->hwaCfg.window = (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                         pHwConfig->hwaCfg.windowSize,
                                                         sizeof(uint32_t));

    if (pHwConfig->hwaCfg.window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA2D_HWA_WINDOW;
        goto exit;
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)pHwConfig->hwaCfg.window,
                            (uint32_t) params->numDopplerChirps,
                            pHwConfig->hwaCfg.windowSize/sizeof(uint32_t),
                            DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_DOPPLER_FFT);
    }

    pHwConfig->radarCube.dataSize = params->numTxAntennas * params->numRangeBins * params->numDopplerBins * params->numRxAntennas * 4;
    pHwConfig->radarCube.data     = (cmplx16ImRe_t *)gMmwMssMCB.radarCube[0].data;
    pHwConfig->radarCube.datafmt  = DPIF_RADARCUBE_FORMAT_2;
    
    pHwConfig->cfarRngDopSnrList = gMmwMssMCB.cfarRngDopSnrList;
    pHwConfig->cfarRngDopSnrListSize = MAX_NUM_DETECTIONS;

    pHwConfig->detObjOutMaxSize = MAX_NUM_DETECTIONS;
    pHwConfig->detObjOut = gMmwMssMCB.dpcAoAObjOut;
    pHwConfig->detObjOutSideInfo = gMmwMssMCB.dpcAoAObjSideInfo;
    
    pHwConfig->detObj2dAzimIdx = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                        pHwConfig->detObjOutMaxSize *sizeof(uint8_t),
                                                        1);
    if (pHwConfig->detObj2dAzimIdx == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_2_AZIM_IDX;
        goto exit;
    }

    pHwConfig->detObjElevationAngle = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                            pHwConfig->detObjOutMaxSize *sizeof(float),
                                                            DPC_OBJDET_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT);
    if (pHwConfig->detObjElevationAngle == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_ELEVATION_ANGLE;
        goto exit;
    }

	/* Allocate buffers for ping and pong paths: */
    pHwConfig->localScratchBufferSizeBytes = DPU_AOAPROCHWA_NUM_LOCAL_SCRATCH_BUFFER_SIZE_BYTES(params->numTxAntennas);
    ind = 0;
    for (ind = 0; ind < DPU_AOAPROCHWA_NUM_LOCAL_SCRATCH_BUFFERS; ind++)
    {
        pHwConfig->localScratchBuffer[ind] = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                         pHwConfig->localScratchBufferSizeBytes,
                                         DPU_AOAPROCHWA_LOCAL_SCRATCH_BYTE_ALIGNMENT);
       if (pHwConfig->localScratchBuffer[ind] == NULL)
       {
           retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA_SCRATCH_BUFFER;
           goto exit;
       }
    }

    /* hwRes - edmaCfg */
    pHwConfig->edmaHandle = gEdmaHandle[0];

    /* For main data processing ping/pong paths */
    pHwConfig->edmaHwaExt[0].chIn.channel =               DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_0;
    pHwConfig->edmaHwaExt[0].chIn.eventQueue =            DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PING_EVENT_QUE;
    pHwConfig->edmaHwaExt[0].chOut.channel =              DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_0;
    pHwConfig->edmaHwaExt[0].chOut.eventQueue =           DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PING_EVENT_QUE;
    pHwConfig->edmaHwaExt[0].stage[0].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_0;
    pHwConfig->edmaHwaExt[0].stage[0].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_1;
    pHwConfig->edmaHwaExt[0].stage[0].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_2;
    pHwConfig->edmaHwaExt[0].stage[1].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_3;
    pHwConfig->edmaHwaExt[0].stage[1].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_4;
    pHwConfig->edmaHwaExt[0].stage[1].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_5;
    pHwConfig->edmaHwaExt[0].stage[1].paramPeakCnt =      DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_12;
    pHwConfig->edmaHwaExt[0].stage[1].paramHwaContinue =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_14;
    pHwConfig->edmaHwaExt[0].eventQueue = 0;

    pHwConfig->edmaHwaExt[1].chIn.channel =               DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_1;
    pHwConfig->edmaHwaExt[1].chIn.eventQueue =            DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PONG_EVENT_QUE;
    pHwConfig->edmaHwaExt[1].chOut.channel =              DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_1;
    pHwConfig->edmaHwaExt[1].chOut.eventQueue =           DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PONG_EVENT_QUE;
    pHwConfig->edmaHwaExt[1].stage[0].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_6;
    pHwConfig->edmaHwaExt[1].stage[0].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_7;
    pHwConfig->edmaHwaExt[1].stage[0].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_8;
    pHwConfig->edmaHwaExt[1].stage[1].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_9;
    pHwConfig->edmaHwaExt[1].stage[1].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_10;
    pHwConfig->edmaHwaExt[1].stage[1].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_11;
    pHwConfig->edmaHwaExt[1].stage[1].paramPeakCnt =      DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_13;
    pHwConfig->edmaHwaExt[1].stage[1].paramHwaContinue =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_15;
    pHwConfig->edmaHwaExt[1].eventQueue = 0;

    /* dynamic config */
    gMmwMssMCB.multiObjBeamFormingCfg.enabled = 0;
    gMmwMssMCB.multiObjBeamFormingCfg.multiPeakThrsScal = 0.5;
    dynCfg->fovAoaCfg                  = &gMmwMssMCB.fovAoaCfg;
    dynCfg->multiObjBeamFormingCfg     = &gMmwMssMCB.multiObjBeamFormingCfg;

    /* Rx compensation coefficients */
    dynCfg->compRxChanCfg = &gMmwMssMCB.compRxChannelBiasCfgMajor;

exit:
return retVal;
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the MacroDoppler processing DPU configurations
*/
int32_t dsp_configParser()
{
    int32_t retVal = 0;
    int32_t pointCloudSize;
    DPIF_MSS_DSS_PreStartCfg *pParam_s = &gMmwMssMCB.dspPreStartCfgLocal;


    /* Allocate point cloud list passed to feature Extraction */
    pointCloudSize                       = DPIF_MAX_RESOLVED_OBJECTS_PER_FRAME * sizeof(FEXTRACT_measurementPoint);
    gMmwMssMCB.pointCloudToFeatExtr         = (FEXTRACT_measurementPoint *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                                   pointCloudSize,
                                                                                                   sizeof(uint32_t));
    if (gMmwMssMCB.pointCloudToFeatExtr == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_POINTT_CLOUD_TO_FEATURE_EXTR;
        goto exit;
    }

    /* Allocate configuration for DSP */
    gMmwMssMCB.dspPreStartCfgShare = (DPIF_MSS_DSS_PreStartCfg *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                     sizeof(DPIF_MSS_DSS_PreStartCfg),
                                                                                     sizeof(uint32_t));
    if (gMmwMssMCB.dspPreStartCfgShare == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DSP_CFG;
        goto exit;
    }

    /* Complete the population of the DSP configuration gMmwMssMCB.objDetDynCfg */

    /* Populate radar processing configuration */
    pParam_s->numFrmPerSlidingWindow = 4;

    pParam_s->numRangeBins    = gMmwMssMCB.numRangeBins;
    pParam_s->rangeFftSize    = gMmwMssMCB.rangeFftSize;
    pParam_s->numTxAntenna    = gMmwMssMCB.numTxAntennas;
    pParam_s->numPhyRxAntenna = gMmwMssMCB.numRxAntennas;
    pParam_s->numAntenna      = pParam_s->numTxAntenna * pParam_s->numPhyRxAntenna;
    if (pParam_s->numTxAntenna > 1)
        pParam_s->mimoModeFlag = 1;
    else
        pParam_s->mimoModeFlag = 0;
    pParam_s->numAdcSamplePerChirp       = gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples;
    pParam_s->dynamicCfarConfig.rangeRes = gMmwMssMCB.rangeStep;
    pParam_s->staticCfarConfig.rangeRes  = gMmwMssMCB.rangeStep;

    // Minor Motion Radar Cube only has 1 chirp per frame due to DC estimation's averaging across all doppler bins
    pParam_s->numChirpPerFrame  = 4;

    if (gMmwMssMCB.timerDrivenDpcMode == 0)
    {
        pParam_s->framePeriod       = gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus / 1000;
    }
    else
    {
        pParam_s->framePeriod       = gMmwMssMCB.sigProcChainCommonCfg.framePeriodicityus / 1000;
    }
    pParam_s->chirpInterval     = (gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpIdleTimeus + gMmwMssMCB.mmWaveCfg.profileComCfg.chirpRampEndTimeus) * 1e-6;
    pParam_s->bandwidth         = gMmwMssMCB.bandwidth;
    pParam_s->centerFreq        = gMmwMssMCB.centerFreq;

    pParam_s->dynamicCfarConfig.dopplerRes = gMmwMssMCB.dopplerStep;
    pParam_s->dynamicCfarConfig.cfarType   = DPIF_RADARDEMO_DETECTIONCFAR_RA_CASOCFAR; // hardcoded, only method can be used in this chain
    pParam_s->dynamicCfarConfig.inputType  = DPIF_RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP; // hardcoded, only method can be used in this chain
    pParam_s->staticCfarConfig.cfarType    = DPIF_RADARDEMO_DETECTIONCFAR_RA_CASOCFARV2; // hardcoded, only method can be used in this chain
    pParam_s->staticCfarConfig.inputType   = DPIF_RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP; // hardcoded, only method can be used in this chain
    pParam_s->maxNumDetObj                 = (uint16_t) DPIF_MAX_RESOLVED_OBJECTS_PER_FRAME;


    gMmwMssMCB.dspPreStartCfgLocal.radarCube = gMmwMssMCB.radarCube[1];

    pParam_s->exportCoarseHeatmap = gMmwMssMCB.dbgGuiMonSel.exportCoarseHeatmap;
    pParam_s->exportRawCfarDetList = gMmwMssMCB.dbgGuiMonSel.exportRawCfarDetList;
    pParam_s->exportZoomInHeatmap = gMmwMssMCB.dbgGuiMonSel.exportZoomInHeatmap;

    pParam_s->disablePointCloudGeneration = 0;
exit:
    return retVal;
}
/**
*  @b Description
*  @n
*    Computes range resolution, lambda/dx, lambda/dz
*/
void mmwDemo_computeProfileParams ()
{
    float                bandwidth, centerFreq, adcStart, slope, startFreq;

    gMmwMssMCB.adcStartTime         = (gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpAdcStartTime) * (1/gMmwMssMCB.adcSamplingRate); //us
    adcStart                        =   (gMmwMssMCB.adcStartTime * 1.e-6);
    startFreq                       =   (float)(gMmwMssMCB.mmWaveCfg.profileTimeCfg.startFreqGHz * 1.e9);
    slope                           =   (float)(gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpSlope * 1.e12);
    bandwidth                       =   (slope * gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples)/(gMmwMssMCB.adcSamplingRate * 1.e6);
    centerFreq                      =   startFreq + bandwidth * 0.5f + adcStart * slope;

    if (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame == 0)
    {
            CLI_write("Error in setting number of bursts in frame\n");
            DebugP_assert(0);
    }

    gMmwMssMCB.rangeStep            =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * (gMmwMssMCB.adcSamplingRate * 1.e6)) /
                                            (2.f * slope * (2*gMmwMssMCB.numRangeBins));

    if (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame > 1)
    {
        /* Burst mode: h_NumOfBurstsInFrame > 1 */
        gMmwMssMCB.dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f *  centerFreq * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.burstPeriodus * 1e-6);
    }
    else
    {
        /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= 2 */
        gMmwMssMCB.dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * centerFreq * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst *
                                            (gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpIdleTimeus + gMmwMssMCB.mmWaveCfg.profileComCfg.chirpRampEndTimeus) * 1e-6);
    }

    /*outParams->dopplerResolution    =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                        (2.f * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame * centerFreq * (gMmwMssMCB.frameCfg.w_BurstPeriodicity));*/
    gMmwMssMCB.bandwidth = bandwidth;
    gMmwMssMCB.centerFreq = centerFreq;

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts == 0.)
    {
        gMmwMssMCB.lambdaOverDistX = 2.0;
    }
    else
    {
        gMmwMssMCB.lambdaOverDistX = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts);
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts == 0.)
    {
        gMmwMssMCB.lambdaOverDistZ = 2.0;
    }
    else
    {
        gMmwMssMCB.lambdaOverDistZ = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts);
    }

    gMmwMssMCB.numDopplerChirps = gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.numFramesPerMinorMode / gMmwMssMCB.numTxAntennas;
    gMmwMssMCB.numDopplerBins = mathUtils_pow2roundup(gMmwMssMCB.numDopplerChirps);
}

/**
*  @b Description
*  @n
*        Function configuring configures DSP for SBR/CPD mode
*/
void mmwDemo_dspConfig()
{
    int32_t retVal = 0;
    MsgIpc_Cfg msgIpcCfg;

    if(!gMmwMssMCB.msgIpcCtrlObj.isMsgIpcInitialized)
    {
        /* Configure IPC */
        msgIpcCfg.msgChanId = 1;
        msgIpcCfg.remoteCoreId = CSL_CORE_ID_C66SS0;
        msgIpcCfg.msgCallback = (IpcNotify_FxnCallback) DPC_mss_MsgHandler;
        msgIpcCfg.arg = NULL;

        MsgIpc_Config(&gMmwMssMCB.msgIpcCtrlObj, &msgIpcCfg);

        /* Create Classifier task semaphore */
        retVal = SemaphoreP_constructBinary(&gMmwMssMCB.classifierTaskSemHandle, 0);
        DebugP_assert(SystemP_SUCCESS == retVal);
        retVal = SemaphoreP_constructBinary(&gMmwMssMCB.classifierTaskSem2Handle, 0);
        DebugP_assert(SystemP_SUCCESS == retVal);
    }
    /* Create Classifier task */
    gDspPointCloudTask = xTaskCreateStatic(MmwDemo_dspPointCloudTask, /* Pointer to the function that implements the task. */
                             "Classifier_task",      /* Text name for the task.  This is to facilitate debugging only. */
                             CLASSIFIER_TASK_STACK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                             NULL,                  /* We are not using the task parameter. */
                             CLASSIFIER_TASK_PRIORITY,          /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                             gDspPointCloudTaskStack,      /* pointer to stack base */
                             &gDspPointCloudTaskObj);         /* pointer to statically allocated task object memory */
    configASSERT(gDspPointCloudTask != NULL);


    /* Parser */
    retVal = dsp_configParser();
    if (retVal < 0)
    {
        CLI_write("Error: Error in setting up DSP Configuration:%d \n", retVal);
        DebugP_assert(0);
    }

    if(!gMmwMssMCB.msgIpcCtrlObj.isMsgIpcInitialized)
    {
        /* Create semaphore to wait for DSP configuration */
        SemaphoreP_constructBinary(&gMmwMssMCB.dspCfgDoneSemaphore, 0);
        /* Sync with DSP */
        MsgIpc_Sync();
        gMmwMssMCB.msgIpcCtrlObj.isMsgIpcInitialized = TRUE;
    }


    /* Copy pre start configuration from local to shared memory location */
    memcpy(gMmwMssMCB.dspPreStartCfgShare, &gMmwMssMCB.dspPreStartCfgLocal, sizeof(DPIF_MSS_DSS_PreStartCfg));

    /* Send DSP configuration */
    MsgIpc_sendMessage(&gMmwMssMCB.msgIpcCtrlObj, DPC_MSS_TO_DSS_PRE_START_CONFIG, (uint32_t) gMmwMssMCB.dspPreStartCfgShare);

    /* Wait for DSP configuration completion */
    SemaphoreP_pend(&gMmwMssMCB.dspCfgDoneSemaphore, SystemP_WAIT_FOREVER);

}



/**
*  @b Description
*  @n
*        Function configuring range processing DPU
*/
void DPC_ObjDet_RngDpuCfg()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = DPC_ObjDet_RngDpuCfg_Parser();
    if (retVal < 0)
    {
        CLI_write("Error in setting up range profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_RangeProcHWA_config(gMmwMssMCB.rangeProcDpuHandle, &rangeProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("Error: RANGE DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    numUsedHwaParamSets = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    
    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function configuring dopplerproc
*/
void DPC_ObjDet_DopplerDpuCfg()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets, numPingPongPath;

    numPingPongPath = 2;

    retVal = DPC_ObjDet_DopplerDpuCfg_Parser();
    if (retVal < 0)
    {
        CLI_write("Error: Error in setting up doppler profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_DopplerProcHWA_config (gMmwMssMCB.dopplerProcDpuHandle, &gDopplerProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("Doppler DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    numUsedHwaParamSets = DPU_DOPPLERPROCHWA_NUM_HWA_PARAMS_FMT2(numPingPongPath);
    
    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function configuring CFAR DPU
*/
void DPC_ObjDet_CfarDpuCfg()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = DPC_ObjDet_CfarDpuCfg_Parser();
    if (retVal < 0)
    {
        CLI_write("Error in setting up CFAR profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_CFARProcHWA_config(gMmwMssMCB.cfarProcDpuHandle, &gCfarProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("CFAR DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    numUsedHwaParamSets = DPU_CFARPROCHWA_NUM_HWA_PARAMS;
    
    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function configuring AOA2D DPU
*/
void DPC_ObjDet_AoaDpuCfg()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = DPC_ObjDet_AoaDpuCfg_Parser();
    if (retVal < 0)
    {
        CLI_write("Error: Error in setting up aoa2d profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_AoAProcHWA_config (gMmwMssMCB.aoa2dProcDpuHandle, &gAoa2dProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("AOA2D DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    numUsedHwaParamSets = DPU_AOAPROCHWA_NUM_HWA_PARAMS;

    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

void DPC_ObjDet_TrackerDpuCfg(void)
{
    int32_t retVal = 0;

    /* Fill sensor position */
    MmwDemo_FillTrackerSensorPositionCfg();

    retVal = DPU_TrackerProc_config(gMmwMssMCB.trackerProcDpuHandle, &gMmwMssMCB.trackerCfg);

    if (retVal < 0)
    {
        CLI_write("Tracker DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }
}


/**
*  @b Description
*  @n
*        Function initiliazing all indvidual DPUs
*/
void DPC_Init()
{
    /* hwa, edma, and DPU initialization*/

    /* Register Frame Start Interrupt */
    if (gMmwMssMCB.timerDrivenDpcMode == 0)
    {
        if(MmwDemo_registerFrameStartInterrupt() != 0){
            CLI_write("Error: Failed to register frame start interrupts\n");
            DebugP_assert(0);
        }
    }

#if 0
    MmwDemo_registerChirpInterrupt();
#endif
#ifdef ENABLE_BURST_INTERRUPT
    MmwDemo_registerBurstInterrupt();
#endif

    int32_t status = SystemP_SUCCESS;

    /* Shared memory pool */
    gMmwMssMCB.L3RamObj.cfg.addr = (void *)&gMmwL3[0];
    gMmwMssMCB.L3RamObj.cfg.size = sizeof(gMmwL3);

    /* Local memory pool */
    gMmwMssMCB.CoreLocalRamObj.cfg.addr = (void *)&gMmwCoreLocMem[0];
    gMmwMssMCB.CoreLocalRamObj.cfg.size = sizeof(gMmwCoreLocMem);
    

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        /* Memory pool for the ID/CPD/SBR */
        HeapP_construct(&gMmwMssMCB.CoreLocalRtosHeapObj, (void *) gMmwCoreLocMem2, MSS_CORE_LOCAL_MEM2_SIZE);
    }

    gHwaHandle = HWA_open(0, NULL, &status);
    if (gHwaHandle == NULL)
    {
        CLI_write("Error: Unable to open the HWA Instance err:%d\n", status);
        DebugP_assert(0);
    }

    DPC_ObjDet_RngDpuInit();
    DPC_ObjDet_DopplerDpuInit();
    DPC_ObjDet_CfarDpuInit();
    DPC_ObjDet_AoaDpuInit();
    DPC_ObjDet_TrackerDpuInit();

}

extern uint32_t gDebugTargetCode;

/**
*  @b Description
*  @n
*    Frame start ISR - used in timer driven RF/DPC mode
*/
void DPC_FrameStartISR(Edma_IntrHandle intrHandle, void *arg)
{
    uint64_t l_demoStartTimeUs;
    unsigned long long ll_startTimeSlowClk;

    uint32_t curCycle;
    MmwDemo_MSS_MCB *mmwMssMCB = (MmwDemo_MSS_MCB *) arg;

    if (mmwMssMCB->timerDrivenArchObj.firstFrameStartIsrStarted == 0)
    {
        ClockP_start(&mmwMssMCB->timerDrivenArchObj.clockObj);
        mmwMssMCB->timerDrivenArchObj.firstFrameStartIsrStarted = 1;
    }
    /* Capture the frame start time using FreeRTOS timer */
    curCycle = Cycleprofiler_getTimeStamp();
    l_demoStartTimeUs = ClockP_getTimeUsec();
    /* Capture the frame start time using the Slow Clock. This is needed when Low power mode is enabled */
    ll_startTimeSlowClk = PRCMSlowClkCtrGet();

    if (gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_DISABLE)
    {
        /* For testing */
        mmwMssMCB->stats.framePeriod_us = (curCycle - mmwMssMCB->stats.frameStartTimeStamp[(mmwMssMCB->stats.frameStartIntCounter-1) & 0x3])/FRAME_REF_TIMER_CLOCK_MHZ;
        mmwMssMCB->stats.frameStartTimeStamp[mmwMssMCB->stats.frameStartIntCounter & 0x3] = curCycle;


        GPIO_pinWriteHigh(gGpioBaseAddrLed, gPinNumLed);
    }
    else
    {
        /* FreeRTOS timer is shutdown during Low Power mode. Hence Slow Clock has to be used when Low power mode is enabled */
        mmwMssMCB->stats.framePeriod_us = round((ll_startTimeSlowClk - mmwMssMCB->stats.frameStartTimeStampSlowClk) * M_TICKS_TO_USEC_SLOWCLK);
    }
    mmwMssMCB->stats.frameStartTimeStampUs = l_demoStartTimeUs;

    mmwMssMCB->stats.frameStartTimeStampSlowClk = ll_startTimeSlowClk;


    if (gDebugTargetCode == 0)
    {
        DebugP_assert(mmwMssMCB->interSubFrameProcToken == 0);
    }

    if(mmwMssMCB->interSubFrameProcToken > 0)
    {
        mmwMssMCB->interSubFrameProcOverflowCntr++;
    }

    mmwMssMCB->interSubFrameProcToken++;

    mmwMssMCB->stats.frameStartIntCounter++;

    // if ((mmwMssMCB->runningMode == DPC_RUNNING_MODE_INDET) && (mmwMssMCB->lowPowerMode == LOW_PWR_MODE_ENABLE))
    // {
    //     //Change to oscilator clock (40MHz)
    //     SOC_rcmSetR5Clock(SOC_RCM_XTAL_CLK_40MHZ,SOC_RCM_XTAL_CLK_40MHZ, SOC_RcmR5ClockSource_OSC_CLK);
    // }

    mmwMssMCB->timerDrivenArchObj.frameStartIntCounter++;
    if (mmwMssMCB->timerDrivenArchObj.numFrames > 0)
    {
        if (mmwMssMCB->timerDrivenArchObj.frameStartIntCounter == mmwMssMCB->timerDrivenArchObj.numFrames)
        {
            ClockP_stop(&mmwMssMCB->timerDrivenArchObj.clockObj);
        }
    }
}


/**
*  @b Description
*  @n

*        For timer driven DPC architecture
*        Frame start event triggers EDMA, that calls Frame start ISR at the begining of the frame.
*/
int32_t mmwDemo_TimerDrivenDpcArchEDMAConfig(DPC_TimerDrivenArchObj *obj)
{
    EDMA_Handle edmaHandle = obj->edmaHandle;
    int32_t                 errorCode = SystemP_SUCCESS;
    uint32_t                edmaReturn;
    int32_t                 idx;
    int32_t                 numParamSets;
    uint32_t                dmaCh, tcc, param, chType;
    uint32_t                baseAddr, regionId;
    EDMACCPaRAMEntry        shadowParam;
    uint16_t                linkChId0;
    uint16_t                linkChId1;


    /* Configure Frame Start EDMA that triggers Frame Start ISR */
    baseAddr = EDMA_getBaseAddr(edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    chType = (uint8_t)EDMA_CHANNEL_TYPE_DMA;
    dmaCh = obj->edmaFrameStart.channel;
    param = obj->edmaFrameStart.channel;
    tcc = obj->edmaFrameStart.channel;
    if ((edmaReturn = DPEDMA_configDummyChannel(edmaHandle, chType, &dmaCh, &tcc, &param)) != SystemP_SUCCESS)
    {
        errorCode = DPC_OBJECTDETECTION_ERROR_TIMER_DRIVEN_DPC_ARCH_CFG;
        goto exit;
    }

    numParamSets = 2;

    /* Program Shadow Param Sets */
    EDMACCPaRAMEntry_init(&shadowParam);

    for (idx = 0; idx < numParamSets; idx++)
    {
        memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
        shadowParam.srcAddr = (uint32_t) obj->dummySrcPtr;
        shadowParam.destAddr = (uint32_t) obj->dummyDstPtr;
        shadowParam.aCnt = 4;
        shadowParam.bCnt = 1;
        if(idx==0)
        {
            shadowParam.cCnt = 1;
        }
        else
        {
            shadowParam.cCnt = obj->numInEvents - 1;
        }
        shadowParam.bCntReload = shadowParam.bCnt;

        shadowParam.srcBIdx = 0;
        shadowParam.destBIdx = 0;

        shadowParam.srcCIdx = 0;
        shadowParam.destCIdx = 0;

        if(idx==0)
        {
            shadowParam.opt          |= (EDMA_OPT_TCINTEN_MASK);
            shadowParam.opt          |=
             (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
             ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
        }
        else
        {
            shadowParam.opt          |=
            (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
             ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
        }

        EDMASetPaRAM(baseAddr,
                      obj->edmaFrameStart.ShadowPramId[idx],
                      &shadowParam);
    }
    /**********************************************/
    /* Link physical channel and param sets       */
    /**********************************************/
    /* Get LinkChan from configuraiton */
    linkChId0 = obj->edmaFrameStart.ShadowPramId[0];
    linkChId1 = obj->edmaFrameStart.ShadowPramId[1];

    /* Link 2 shadow Param sets */
    if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                          param,
                                          linkChId0)) != SystemP_SUCCESS)
    {
        goto exit;
    }
    if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                          linkChId0,
                                          linkChId1)) != SystemP_SUCCESS)
    {
        goto exit;
    }
    if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                          linkChId1,
                                          linkChId0)) != SystemP_SUCCESS)
    {
        goto exit;
    }

    /********************************/
    /* Register ISR                 */
    /********************************/
    obj->intrObj.tccNum = obj->edmaFrameStart.channel;
    obj->intrObj.cbFxn  = (Edma_EventCallback) DPC_FrameStartISR;
    obj->intrObj.appData = (void *) &gMmwMssMCB;
    errorCode = EDMA_registerIntr(edmaHandle, &obj->intrObj);
    if (errorCode != SystemP_SUCCESS)
    {
        goto exit;
    }

    /********************************/
    /* Bring in the first param set */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);
    if (edmaReturn != TRUE)
    {
       errorCode = DPC_OBJECTDETECTION_ERROR_TIMER_DRIVEN_DPC_ARCH_CFG;
       goto exit;
    }

    /********************************/
    /* Enable event                 */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_EVENT);
    if (edmaReturn != TRUE)
    {
       errorCode = DPC_OBJECTDETECTION_ERROR_TIMER_DRIVEN_DPC_ARCH_CFG;
       goto exit;
    }

exit:
    return errorCode;
}

/**
*  @b Description
*  @n

*        Configuration for Prolonged (Continuous) bursting mode. EDMA configuration. Configures 
*        one EDMA channel with two shadow channels, EDMA channel is triggered by chirp available event, 
*        the first shadow PARAM set passses through the evnts to range DPU, the second one is dummy 
*        and gates the remaining events.
*/
int32_t DPC_ProlonedBurstingConfig(DPC_prolonedBurstingObj *obj)
{
    EDMA_Handle edmaHandle = obj->edmaHandle;
    int32_t                 errorCode = SystemP_SUCCESS;
    uint32_t                edmaReturn;
    int32_t                 idx;
    int32_t                 numParamSets;
    uint32_t                dmaCh, tcc, param, chType;
    uint32_t                baseAddr, regionId;
    EDMACCPaRAMEntry        shadowParam;
    uint16_t                linkChId0;
    uint16_t                linkChId1;
    uint16_t                linkChId2;


    baseAddr = EDMA_getBaseAddr(edmaHandle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(edmaHandle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    chType = (uint8_t)EDMA_CHANNEL_TYPE_DMA;
    dmaCh = obj->edmaEvtSplit.channel;
    param = obj->edmaEvtSplit.channel;
    tcc = obj->edmaChainChannel;
    if ((edmaReturn = DPEDMA_configDummyChannel(edmaHandle, chType, &dmaCh, &tcc, &param)) != SystemP_SUCCESS)
    {
        errorCode = DPC_OBJECTDETECTION_ERROR_PROLONGED_BURSTING_MODE_CFG;
        goto exit;
    }

    /* Program Shadow Param Sets */
    EDMACCPaRAMEntry_init(&shadowParam);

    if (obj->startPassThroughEventIdx == 0)
    {
        numParamSets = 2;

        for (idx = 0; idx < numParamSets; idx++)
        {
            memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
            shadowParam.srcAddr = (uint32_t) obj->dummySrcPtr;
            shadowParam.destAddr = (uint32_t) obj->dummyDstPtr;
            shadowParam.aCnt = 4;
            shadowParam.bCnt = 1;
            if(idx==0)
            {
                shadowParam.cCnt = obj->numOutEvents;
            }
            else
            {
                shadowParam.cCnt = obj->numInEvents - obj->numOutEvents;
            }
            shadowParam.bCntReload = shadowParam.bCnt;

            shadowParam.srcBIdx = 0;
            shadowParam.destBIdx = 0;

            shadowParam.srcCIdx = 0;
            shadowParam.destCIdx = 0;

            if(idx==0)
            {
                shadowParam.opt          |=
                (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
                 ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                 ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
            }
            else
            {
                shadowParam.opt          |=
                (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                 ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
            }

            EDMASetPaRAM(baseAddr,
                          obj->edmaEvtSplit.ShadowPramId[idx],
                          &shadowParam);
        }
        /**********************************************/
        /* Link physical channel and param sets       */
        /**********************************************/
        /* Get LinkChan from configuraiton */
        linkChId0 = obj->edmaEvtSplit.ShadowPramId[0];
        linkChId1 = obj->edmaEvtSplit.ShadowPramId[1];

        /* Link 2 shadow Param sets */
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              param,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId0,
                                              linkChId1)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId1,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else if ((obj->numInEvents - obj->startPassThroughEventIdx - obj->numOutEvents) > 0)
    {
        numParamSets = 3;
        for (idx = 0; idx < numParamSets; idx++)
        {
            memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
            shadowParam.srcAddr = (uint32_t) obj->dummySrcPtr;
            shadowParam.destAddr = (uint32_t) obj->dummyDstPtr;
            shadowParam.aCnt = 4;
            shadowParam.bCnt = 1;
            if(idx==0)
            {
                shadowParam.cCnt = obj->startPassThroughEventIdx;
            }
            else if(idx==1)
            {
                shadowParam.cCnt = obj->numOutEvents;
            }
            else
            {
                shadowParam.cCnt = obj->numInEvents - obj->startPassThroughEventIdx - obj->numOutEvents;
            }
            shadowParam.bCntReload = shadowParam.bCnt;

            shadowParam.srcBIdx = 0;
            shadowParam.destBIdx = 0;

            shadowParam.srcCIdx = 0;
            shadowParam.destCIdx = 0;

            if ((idx==0) || (idx==2))
            {
                shadowParam.opt          |=
                (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                 ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
            }
            else
            {
                shadowParam.opt          |=
                (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
                 ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                 ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
            }

            EDMASetPaRAM(baseAddr,
                          obj->edmaEvtSplit.ShadowPramId[idx],
                          &shadowParam);
        }
        /**********************************************/
        /* Link physical channel and param sets       */
        /**********************************************/
        /* Get LinkChan from configuraiton */
        linkChId0 = obj->edmaEvtSplit.ShadowPramId[0];
        linkChId1 = obj->edmaEvtSplit.ShadowPramId[1];
        linkChId2 = obj->edmaEvtSplit.ShadowPramId[2];

        /* Link 2 shadow Param sets */
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              param,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId0,
                                              linkChId1)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId1,
                                              linkChId2)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId2,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else if (obj->numInEvents == (obj->startPassThroughEventIdx - obj->numOutEvents))
    {
        numParamSets = 2;

        for (idx = 0; idx < numParamSets; idx++)
        {
            memset(&shadowParam, 0, sizeof(EDMACCPaRAMEntry));
            shadowParam.srcAddr = (uint32_t) obj->dummySrcPtr;
            shadowParam.destAddr = (uint32_t) obj->dummyDstPtr;
            shadowParam.aCnt = 4;
            shadowParam.bCnt = 1;
            if(idx==0)
            {
                shadowParam.cCnt = obj->numInEvents - obj->numOutEvents;
            }
            else
            {
                shadowParam.cCnt = obj->numOutEvents;
            }
            shadowParam.bCntReload = shadowParam.bCnt;

            shadowParam.srcBIdx = 0;
            shadowParam.destBIdx = 0;

            shadowParam.srcCIdx = 0;
            shadowParam.destCIdx = 0;

            if(idx==0)
            {
                shadowParam.opt          |=
                (((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                 ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
            }
            else
            {
                shadowParam.opt          |=
                (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK |
                 ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) |
                 ((((uint32_t)EDMA_SYNC_AB) << EDMA_OPT_SYNCDIM_SHIFT) & EDMA_OPT_SYNCDIM_MASK));
            }

            EDMASetPaRAM(baseAddr,
                          obj->edmaEvtSplit.ShadowPramId[idx],
                          &shadowParam);
        }
        /**********************************************/
        /* Link physical channel and param sets       */
        /**********************************************/
        /* Get LinkChan from configuraiton */
        linkChId0 = obj->edmaEvtSplit.ShadowPramId[0];
        linkChId1 = obj->edmaEvtSplit.ShadowPramId[1];

        /* Link 2 shadow Param sets */
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              param,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId0,
                                              linkChId1)) != SystemP_SUCCESS)
        {
            goto exit;
        }
        if ((errorCode = DPEDMA_linkParamSets(edmaHandle,
                                              linkChId1,
                                              linkChId0)) != SystemP_SUCCESS)
        {
            goto exit;
        }
    }
    else
    {
       errorCode = DPC_OBJECTDETECTION_ERROR_PROLONGED_BURSTING_MODE_CFG;
       goto exit;
    }

    /********************************/
    /* Bring in the first param set */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);
    if (edmaReturn != TRUE)
    {
       errorCode = DPC_OBJECTDETECTION_ERROR_PROLONGED_BURSTING_MODE_CFG;
       goto exit;
    }

    /********************************/
    /* Enable event                 */
    /********************************/
    edmaReturn = EDMAEnableTransferRegion(baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_EVENT);
    if (edmaReturn != TRUE)
    {
       errorCode = DPC_OBJECTDETECTION_ERROR_PROLONGED_BURSTING_MODE_CFG;
       goto exit;
    }

exit:
    return errorCode;
}

/**
*  @b Description
*  @n

*        Function configuring all DPUs
*/
void DPC_Config()
{

    int32_t retVal;

    /*TODO Cleanup: MMWLPSDK-237*/
    
    DPC_ObjDet_MemPoolReset(&gMmwMssMCB.L3RamObj);
    DPC_ObjDet_MemPoolReset(&gMmwMssMCB.CoreLocalRamObj);
    DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(&gMmwMssMCB.HwaDmaChanPoolObj);
    DPC_ObjDet_HwaWinRamMemoryPoolReset(&gMmwMssMCB.HwaWinRamMemoryPoolObj);

    if (gMmwMssMCB.adcLogging.enable == 1)
    {
        if(MmwDemo_registerChirpAvailableInterrupts() != 0)
        {
            CLI_write("Failed to register chirp available interrupts\n");
            DebugP_assert(0);
        }
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        /* Reset memory for feature extraction */
        HeapP_construct(&gMmwMssMCB.CoreLocalRtosHeapObj, (void *) gMmwCoreLocMem2, MSS_CORE_LOCAL_MEM2_SIZE);
    }
    gMmwMssMCB.dpcAoAObjOut = (DPIF_PointCloudCartesian *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudCartesian),
                                                                                       DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT);
    if (gMmwMssMCB.dpcAoAObjOut == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    gMmwMssMCB.dpcAoAObjSideInfo = (DPIF_PointCloudSideInfo *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudSideInfo),
                                                                                       DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT);
    if (gMmwMssMCB.dpcAoAObjSideInfo == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    /*Allocate memory for aoa output conversion to CartesianExt struct*/
    gMmwMssMCB.dpcAoAObjOutCartExt = (DPIF_PointCloudCartesianExt *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                            MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudCartesianExt),
                                                                                            DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT);

    if (gMmwMssMCB.dpcAoAObjOutCartExt == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    /* Select active antennas from available antennas and calculate number of antennas rows and columns */
    MmwDemo_calcActiveAntennaGeometry();

    /* Configure DPUs */
    mmwDemo_computeProfileParams();

    /* SBR or CPD DETECTION MODE */
    // Pedrhom
    gMmwMssMCB.numFramesPerMinorMode = 4;
    DPC_ObjDet_RngDpuCfg();
    DPC_ObjDet_DopplerDpuCfg();
    DPC_ObjDet_CfarDpuCfg();
    DPC_ObjDet_AoaDpuCfg();
    mmwDemo_dspConfig();
    DPC_ObjDet_TrackerDpuCfg();

    if(gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
    {
        retVal = MmwDemo_rangeBiasRxChPhaseMeasureConfig();
        if (retVal != 0)
        {
            CLI_write("DPC configuration: Invalid Rx channel compensation procedure configuration \n");
            DebugP_assert(0);
        }
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {

        /* Report RAM usage */
        gMmwMssMCB.memUsage.CoreLocalRamUsage = DPC_ObjDet_MemPoolGetMaxUsage(&gMmwMssMCB.CoreLocalRamObj);
        gMmwMssMCB.memUsage.L3RamUsage = DPC_ObjDet_MemPoolGetMaxUsage(&gMmwMssMCB.L3RamObj);
        
        gMmwMssMCB.memUsage.L3RamTotal = gMmwMssMCB.L3RamObj.cfg.size;
        gMmwMssMCB.memUsage.CoreLocalRamTotal = gMmwMssMCB.CoreLocalRamObj.cfg.size;
    
        if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_DISABLE)
        {
            DebugP_log(" ========== Memory Stats ==========\n");
            DebugP_log("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");

            DebugP_log("%20s %12d %12d %12d\n", "L3",
                      sizeof(gMmwL3),
                      gMmwMssMCB.memUsage.L3RamUsage,
                      sizeof(gMmwL3) - gMmwMssMCB.memUsage.L3RamUsage);

            DebugP_log("%20s %12d %12d %12d\n", "Local",
                      sizeof(gMmwCoreLocMem),
                      gMmwMssMCB.memUsage.CoreLocalRamUsage,
                      sizeof(gMmwCoreLocMem) - gMmwMssMCB.memUsage.CoreLocalRamUsage);

            DebugP_log("%20s %12d %12d %12d\n", "Local2",
                      sizeof(gMmwCoreLocMem2),
                      coreLocalRtosHeap_memUsage(),
                      sizeof(gMmwCoreLocMem2) - coreLocalRtosHeap_memUsage());

        }
    }

}

void createMinorModeCube(cmplx16ImRe_t *minorModeRadarCube, cmplx16ImRe_t *majorModeRadarCube, 
                            int32_t numRangeBins, int32_t numDopplerChirps, uint8_t numTxAntennas, uint8_t numRxAntennas, 
                            uint8_t frameCount, uint8_t numFramesPerMinorMode, int32_t log2numDopplerChirps)
{
    int32_t InputBinIdx, OutputBinIdx;
    int32_t minorModeFrameCnt = frameCount % numFramesPerMinorMode;
    int32_t minorModeChirpSumReal;
    int32_t minorModeChirpSumImag;

    for (int rangeIdx = 0; rangeIdx < numRangeBins; rangeIdx++)
    {
        for (int txIdx = 0;  txIdx < numTxAntennas; txIdx++)
        {
            for (int rxIdx = 0; rxIdx < numRxAntennas; rxIdx++)
            {
                // Index to the minorMode radar cube. 
                OutputBinIdx = rxIdx + txIdx*numRxAntennas + minorModeFrameCnt*numTxAntennas*numRxAntennas   
                            + rangeIdx*numTxAntennas*numRxAntennas*numFramesPerMinorMode;


                for (int chirpIdx = 0; chirpIdx < numDopplerChirps; chirpIdx++)
                {
                    InputBinIdx = rxIdx + txIdx*numRxAntennas + chirpIdx*numTxAntennas*numRxAntennas 
                        + rangeIdx*numTxAntennas*numRxAntennas*numDopplerChirps;
                    
                    if (chirpIdx == 0)
                    {
                        minorModeChirpSumReal = (int32_t)majorModeRadarCube[InputBinIdx].real;
                        minorModeChirpSumImag = (int32_t)majorModeRadarCube[InputBinIdx].imag;
                    }
                    else
                    {
                        minorModeChirpSumReal += (int32_t) majorModeRadarCube[InputBinIdx].real;
                        minorModeChirpSumImag += (int32_t) majorModeRadarCube[InputBinIdx].imag;
                    }
                }
                minorModeRadarCube[OutputBinIdx].real = (int16_t) minorModeChirpSumReal >> log2numDopplerChirps;
                minorModeRadarCube[OutputBinIdx].imag = (int16_t) minorModeChirpSumImag >> log2numDopplerChirps;
            }
        }
    }
}

void MmwDemo_FillTrackerSensorPositionCfg()
{

    /*populate sensor position configuration*/
    memcpy(&gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorPosition, &gMmwMssMCB.sceneryParams.sensorPosition, sizeof(GTRACK_sensorPosition));

    /*populate sensor orientation configuration*/
    memcpy(&gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorOrientation, &gMmwMssMCB.sceneryParams.sensorOrientation, sizeof(GTRACK_sensorOrientation));

    /*demo parameters*/
    gMmwMssMCB.trackerCfg.staticCfg.sensorAzimuthTilt   = gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorOrientation.azimTilt * 3.1415926f / 180.;
    gMmwMssMCB.trackerCfg.staticCfg.sensorElevationTilt = gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorOrientation.elevTilt * 3.1415926f / 180.;
    gMmwMssMCB.trackerCfg.staticCfg.sensorHeight        = gMmwMssMCB.trackerCfg.staticCfg.sceneryParams.sensorPosition.z;
}
        
volatile uint64_t gTest;
//#define PROFILE_INTRUSION_DPUS
#ifdef PROFILE_INTRUSION_DPUS
volatile uint32_t gProfileTime[8][5];
volatile uint32_t gProfileTimeInd = 0;
volatile uint32_t gStartTime;
volatile uint32_t gEndTime;
#endif

volatile cmplx16ImRe_t *gRngBin28VecPtr;
volatile cmplx16ImRe_t gRngBin28Vec[64];
volatile uint32_t gRngBin28VecIdx = 0;

volatile uint32_t gMacroDopplerDpuTime[16];
volatile uint32_t gMacroDopplerDpuTimeIdx = 0;
/**
 *  @b Description
 *  @n  DPC processing chain execute function.
 *
 */
/* Spatial3D: RANGE-WINDOW zero-Doppler cube extraction (Phase 1, track-independent).
 *
 * The on-chip fall state machine (velZ ARM -> floor CONFIRM -> BURST) was removed: the
 * People_Tracking tracker FREEZES the track at the moment of a fall, so a track-gated
 * burst never fires exactly when the fall second-check needs it. Extraction is now a
 * pure function of a RANGE BIN, armed by the server via the `cubeQuery` CLI command
 * (see MmwDemo_CLICubeQuery). This ignores the tracker entirely, so a lost or frozen
 * track can no longer suppress the cube -> a body on the floor (no track) still yields
 * TLV 320. The fall DECISION (RR / below-floor energy over the returned cube) stays
 * server-side.
 *
 * Fills gMmwMssMCB.tbcEntries[] with the center bin +- halfWin coherent-mean-over-chirp
 * 16-antenna vectors from radarCube[0] (FORMAT_2: [range][chirp][virtAnt]); tid = 0
 * placeholder (no owning track). Sets tbcNumEntries/tbcNumVirtAnt. Called every frame
 * from DPC_Execute while radarCube[0] still holds the current frame. */
static void MmwDemo_tbcExtractBin(int32_t cbin, uint16_t half)
{
    uint16_t nVirt  = (uint16_t)(gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas);
    uint16_t nChirp = (uint16_t)gMmwMssMCB.numDopplerChirps;
    cmplx16ImRe_t *cube = (cmplx16ImRe_t *)gMmwMssMCB.radarCube[0].data;
    float dR = gMmwMssMCB.rangeStep;
    uint16_t e = 0;
    int32_t  b;

    gMmwMssMCB.tbcNumEntries = 0;
    gMmwMssMCB.tbcNumVirtAnt = nVirt;
    if (cube == NULL || nVirt == 0 || nVirt > TBC_MAX_VIRT_ANT || nChirp == 0 || dR <= 0.0f)
    {
        return;
    }

    for (b = cbin - (int32_t)half; b <= cbin + (int32_t)half && e < TBC_MAX_ENTRIES; b++)
    {
        MmwDemo_TrackBinEntry *ent;
        uint32_t base;
        uint16_t a;
        if (b < 0 || b >= (int32_t)gMmwMssMCB.numRangeBins)
        {
            continue;
        }
        ent = &gMmwMssMCB.tbcEntries[e];
        ent->tid         = 0;              /* range-query: no owning track */
        ent->rangeBin    = (uint16_t)b;
        ent->range_m     = (float)b * dR;
        ent->velMag_mmps = 0;
        base = (uint32_t)b * nChirp * nVirt;
        for (a = 0; a < nVirt; a++)
        {
            int32_t accRe = 0, accIm = 0;
            uint16_t c;
            for (c = 0; c < nChirp; c++)
            {
                cmplx16ImRe_t *s = &cube[base + (uint32_t)c * nVirt + a];
                accRe += s->real;
                accIm += s->imag;
            }
            ent->vec[a].real = (int16_t)(accRe / (int32_t)nChirp);
            ent->vec[a].imag = (int16_t)(accIm / (int32_t)nChirp);
        }
        e++;
    }
    gMmwMssMCB.tbcNumEntries = e;
}

/* PosePointGet over the tracker's Cartesian input set (read in place, no copy).
 * CartExt.snr is int16 in 0.1 dB steps; classes.zip trained on dB -> x0.1. */
static void MmwDemo_poseGetPoint(const void *ctx, uint32_t i,
                                 float *x, float *y, float *z, float *snr)
{
    const DPIF_PointCloudCartesianExt *p = (const DPIF_PointCloudCartesianExt *)ctx + i;
    *x   = p->x;
    *y   = p->y;
    *z   = p->z;
    *snr = (float)p->snr * 0.1f;
}

void DPC_Execute(){
    int32_t retVal;
    int32_t errCode = 0;
    int32_t i;

    DPU_RangeProcHWA_OutParams outParms;
    DPU_DopplerProcHWA_OutParams outParmsDoppler;
    DPU_CFARProcHWA_OutParams outParmsCfar;
    DPU_AoAProcHWA_OutParams outParmsAoa2d;
    DSSHWACCRegs *ctrlBaseAddr = (DSSHWACCRegs *)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    
    DPC_ObjectDetection_ExecuteResult *result = &gMmwMssMCB.dpcResult;
    uint32_t numDetectedPoints;

    uint8_t frameCount = 0;
    
    /* give initial trigger for the first frame */
    errCode = DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle,
                                    DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    if(errCode < 0)
    {
        CLI_write("Error: Range control execution failed [Error code %d]\n", errCode);
    }

    result->objOut = gMmwMssMCB.dpcAoAObjOut;
    result->objOutSideInfo = gMmwMssMCB.dpcAoAObjSideInfo;
    result->rngDopplerHeatMap = (uint16_t *) gMmwMssMCB.detMatrix;

    /* Send signal to CLI task that this is ready */
    SemaphoreP_post(&gMmwMssMCB.dpcTaskConfigDoneSemHandle);

    while(true){

        memset((void *)&outParms, 0, sizeof(DPU_RangeProcHWA_OutParams));
        retVal = DPU_RangeProcHWA_process(gMmwMssMCB.rangeProcDpuHandle, &outParms);
        if(retVal != 0){
            CLI_write("DPU_RangeProcHWA_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        // DC Estimation here
        gMmwMssMCB.frameCount = frameCount;
        createMinorModeCube((cmplx16ImRe_t *) gMmwMssMCB.radarCube[1].data, 
            (cmplx16ImRe_t *) gMmwMssMCB.radarCube[0].data, 
            gMmwMssMCB.numRangeBins,
            gMmwMssMCB.numDopplerChirps,
            gMmwMssMCB.numTxAntennas,
            gMmwMssMCB.numRxAntennas,
            gMmwMssMCB.frameCount,
            gMmwMssMCB.numFramesPerMinorMode,
            gMmwMssMCB.log2numDopplerChirps
            );

        MsgIpc_sendMessage(&gMmwMssMCB.msgIpcCtrlObj, DPC_MSS_TO_DSS_RADAR_CUBE_READY, (uint32_t) &gMmwMssMCB.radarCube[1]);

        frameCount++;

        if (frameCount == 4)
        {
            frameCount = 0;

        }

        GPIO_pinWriteLow(gGpioBaseAddrLed, gPinNumLed);
#if 1
        /* Read the temperature */
        MMWave_getTemperatureReport(&gTempStats);
#endif
        /* Chirping finished start interframe processing */
        gMmwMssMCB.stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();
#ifdef PROFILE_INTRUSION_DPUS
        gStartTime = gMmwMssMCB.stats.interFrameStartTimeStamp;
#endif
        DPC_ObjectDetection_Profile(&gMmwMssMCB.stats.chirpingCompletion);
        gMmwMssMCB.stats.chirpingTime_us = gMmwMssMCB.stats.chirpingCompletion.timeInUsec;
        
        //  /* Procedure for range bias measurement and Rx channels gain/phase offset measurement */
        if(gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
        {
            MmwDemo_rangeBiasRxChPhaseMeasure();
        }

        /* Doppler DPU */
        memset((void *)&outParmsDoppler, 0, sizeof(DPU_DopplerProcHWA_OutParams));
        retVal = DPU_DopplerProcHWA_process(gMmwMssMCB.dopplerProcDpuHandle, &outParmsDoppler);
        if(retVal != 0){
            CLI_write("DPU_DopplerProc_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        // /********* TODO: Known Errata - MMWSOC_IWRL68XX-1900. Dynamic clock gating disabled for HWA CFAR engine ************/
        CSL_FINSR(ctrlBaseAddr->HWACCREG1,
                    HWACCREG1_ACCDYNCLKEN_BIT_END,
                    HWACCREG1_ACCDYNCLKEN_BIT_START,
                    0x0U);
        // /******************************************************************************************************************/                    
        
        /* CFAR DPU */
        numDetectedPoints = 0;
        memset((void *)&outParmsCfar, 0, sizeof(DPU_CFARProcHWA_OutParams));
        
        retVal = DPU_CFARProcHWA_process(gMmwMssMCB.cfarProcDpuHandle,
                                         &outParmsCfar);
        numDetectedPoints = outParmsCfar.numCfarDetectedPoints;

        result->numObjOut = numDetectedPoints;
        
        if(retVal != 0){
            CLI_write("DPU_CFARProcHWA_process failed with error code %d", retVal);
            DebugP_assert(0);
        }
        if(gMmwMssMCB.multiObjBeamFormingCfg.enabled == 0)
        {
            /********* TODO: Known Errata - MMWSOC_IWRL68XX-1900. Dynamic clock gating disabled for HWA CFAR engine ************/
            CSL_FINSR(ctrlBaseAddr->HWACCREG1,
                        HWACCREG1_ACCDYNCLKEN_BIT_END,
                        HWACCREG1_ACCDYNCLKEN_BIT_START,
                        0x1U);
            /******************************************************************************************************************/
        }
        
        /* Prepare range gates for AoA */
        memset((void *)&outParmsAoa2d, 0, sizeof(DPU_AoAProcHWA_OutParams));
        
        retVal = DPU_AoAProcHWA_process(gMmwMssMCB.aoa2dProcDpuHandle,
                                       numDetectedPoints,
                                       &outParmsAoa2d);
        if(retVal != 0){
            CLI_write("DPU_Aoa2dProc_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        result->numObjOut = outParmsAoa2d.numAoADetectedPoints;
        gMmwMssMCB.numDetectedPointsMajor = result->numObjOut;

        if(gMmwMssMCB.multiObjBeamFormingCfg.enabled == 1)
        {
            /********* TODO: Known Errata - MMWSOC_IWRL68XX-1900. Dynamic clock gating disabled for HWA CFAR engine ************/
            CSL_FINSR(ctrlBaseAddr->HWACCREG1,
                        HWACCREG1_ACCDYNCLKEN_BIT_END,
                        HWACCREG1_ACCDYNCLKEN_BIT_START,
                        0x1U);
            /******************************************************************************************************************/
        }

        if(gMmwMssMCB.guiMonSel.pointCloud == 1)
        {
            for(i=0; i < result->numObjOut; i++)
            {
                result->objOutSideInfo[i].snr = (int16_t) (10. * result->objOutSideInfo[i].snr); //steps of 0.1dB
                result->objOutSideInfo[i].noise = (int16_t) (10. * result->objOutSideInfo[i].noise); //steps of 0.1dB
            }
        }


#ifdef PROFILE_INTRUSION_DPUS
        gEndTime = Cycleprofiler_getTimeStamp();
        gProfileTime[gProfileTimeInd][0] = gEndTime - gStartTime;
        gStartTime = gEndTime;
#endif

        /* Interframe processing finished */
        // gMmwMssMCB.stats.ProcessingEndTimeStampUs = ClockP_getTimeUsec();
        // gMmwMssMCB.outStats.interFrameProcessingTimeUs = (gMmwMssMCB.stats.ProcessingEndTimeStampUs - gMmwMssMCB.stats.ProcessingStartTimeStampUs);

        /* If ADC logging via LVDS is enabled, Pend for completion of session, generally this will not wait
        * because of time spent doing inter-frame processing is expected to
        * be bigger than the transmission of the session */
        if (gMmwMssMCB.adcLogging.enable == 1)
        {
            SemaphoreP_pend(&gMmwMssMCB.lvdsStream.frameDoneSemHandle, SystemP_WAIT_FOREVER);
            if(gMmwMssMCB.lvdsStream.frameDoneCount == (gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame))
            {
                gMmwMssMCB.lvdsStream.frameDoneCount = 0;
            }
            else 
            {
                CLI_write("Error: Some chirps are not transmitted successfully through LVDS\n");
                DebugP_assert(0);
            }
        }
        
        // Compile Minor Motion Points
        MmwDemo_dspPointCloudTask();

        // Tracker DPU
        if (gMmwMssMCB.trackerCfg.staticCfg.trackerEnabled)
        {

            // Major
            for (i = 0; i < result->numObjOut; i++)
            {
                gMmwMssMCB.dpcAoAObjOutCartExt[i].x        = gMmwMssMCB.dpcAoAObjOut[i].x;
                gMmwMssMCB.dpcAoAObjOutCartExt[i].y        = gMmwMssMCB.dpcAoAObjOut[i].y;
                gMmwMssMCB.dpcAoAObjOutCartExt[i].z        = gMmwMssMCB.dpcAoAObjOut[i].z;
                gMmwMssMCB.dpcAoAObjOutCartExt[i].velocity = gMmwMssMCB.dpcAoAObjOut[i].velocity;
                gMmwMssMCB.dpcAoAObjOutCartExt[i].snr      = (int16_t)(gMmwMssMCB.dpcAoAObjSideInfo[i].snr); // steps of 0.1dB
                gMmwMssMCB.dpcAoAObjOutCartExt[i].noise    = (int16_t)(gMmwMssMCB.dpcAoAObjSideInfo[i].noise); // steps of 0.1dB
            }

            // Convert Minor Motion point cloud to Cartesian, then add point cloud to major motion set to send to tracker
            sphericalToCartesianMinorMotionPointCloud(gMmwMssMCB.pointCloudToUart ,gMmwMssMCB.numDetectedPointsMajor,gMmwMssMCB.numDetectedPointsMinor);

            retVal = DPU_TrackerProc_process(gMmwMssMCB.trackerProcDpuHandle,
                                                gMmwMssMCB.numDetectedPointsMinor + gMmwMssMCB.numDetectedPointsMajor,
                                                gMmwMssMCB.dpcAoAObjOutCartExt,
                                                &result->trackerOutParams);
            if (retVal != 0)
            {
                CLI_write("DPU_TrackerProc_process failed with error code %d", retVal);
                DebugP_assert(0);
            }

            /* Spatial3D: per-track pose classification. Runs here where both the
             * fresh target list and this frame's Cartesian point set are valid.
             * Builds per-track kinematics + the major-motion Cartesian points,
             * feeds PoseMlp_process (per-track 8-frame ring buffer + folded MLP),
             * and stashes the results for TLV 321. Auxiliary fall leg; primary
             * fall decision stays server-side. */
            if (gMmwMssMCB.poseEnable)
            {
                uint32_t nT = result->trackerOutParams.numTargets;
                trackerProc_Target *tl = (trackerProc_Target *)result->trackerOutParams.tList;
                /* Only the tiny per-track kinematics is copied (256 B). The point
                 * set is read IN PLACE from dpcAoAObjOutCartExt via MmwDemo_poseGetPoint
                 * -- no ~32 KB PosePoint scratch (which would also risk TCMA). */
                static PoseTrackKin poseKin[POSE_MAX_TRACKS]
                    __attribute__((section(".bss.pose")));
                /* Feed BOTH major + minor Cartesian points -- exactly the set the
                 * tracker gets. dpcAoAObjOutCartExt holds major [0..Major) then
                 * minor [Major..Major+Minor) (sphericalToCartesianMinorMotionPointCloud).
                 * A still/lying person is almost all MINOR motion, so major-only
                 * (numObjOut) starves the pose legs after a fall -> they never
                 * confirm sustained-down. */
                uint32_t nP = gMmwMssMCB.numDetectedPointsMajor +
                              gMmwMssMCB.numDetectedPointsMinor;
                uint32_t i;

                if (nT > POSE_MAX_TRACKS) nT = POSE_MAX_TRACKS;

                for (i = 0; i < nT; i++)
                {
                    poseKin[i].tid  = tl[i].tid;
                    poseKin[i].posX = tl[i].posX;
                    poseKin[i].posY = tl[i].posY;
                    poseKin[i].posZ = tl[i].posZ;
                    poseKin[i].velY = tl[i].velY;
                    poseKin[i].velZ = tl[i].velZ;
                    poseKin[i].accY = tl[i].accY;
                    poseKin[i].accZ = tl[i].accZ;
                }
                gMmwMssMCB.poseNumResults =
                    (uint16_t)PoseMlp_process(poseKin, nT,
                                              gMmwMssMCB.dpcAoAObjOutCartExt, nP,
                                              MmwDemo_poseGetPoint,
                                              gMmwMssMCB.poseResults);
            }
            else
            {
                gMmwMssMCB.poseNumResults = 0;
            }

        }

        /* Spatial3D: range-window cube burst (server `cubeQuery`, track-independent).
         * Must run HERE - radarCube[0] still holds this frame; the next frame's range
         * proc is triggered just below (which starts overwriting it). While a query is
         * armed, extract bin +- halfWin and count the frame down; disarm at zero. */
        /* GUARD (cubeGuardCfg token bucket): refill CONTINUOUSLY every frame, capped at capacity.
         * capacity_milli = tbcBudgetFrames * 1000. No cliff -- a drained bucket recovers smoothly. */
        {
            int32_t capMilli = (int32_t)gMmwMssMCB.tbcBudgetFrames * 1000;
            gMmwMssMCB.tbcTokensMilli += gMmwMssMCB.tbcRefillMilli;
            if (gMmwMssMCB.tbcTokensMilli > capMilli)
            {
                gMmwMssMCB.tbcTokensMilli = capMilli;
            }
            gMmwMssMCB.tbcTokenHbCtr++;    /* one tick/frame -> TLV 322 heartbeat at 300 (transmit) */
        }
        if (gMmwMssMCB.tbcQueryActive && gMmwMssMCB.tbcQueryFramesLeft > 0
                && gMmwMssMCB.tbcTokensMilli >= 1000)
        {
            MmwDemo_tbcExtractBin((int32_t)gMmwMssMCB.tbcQueryBin,
                                  gMmwMssMCB.tbcQueryHalfWin);
            gMmwMssMCB.tbcTokensMilli -= 1000;    /* spend one token (=one cube-frame) */
            if (--gMmwMssMCB.tbcQueryFramesLeft <= 0)
            {
                gMmwMssMCB.tbcQueryActive = 0;   /* burst complete -> stop */
            }
        }
        else
        {
            gMmwMssMCB.tbcQueryActive = 0;        /* done / no tokens -> stop */
            gMmwMssMCB.tbcNumEntries  = 0;       /* no query -> emit no TLV 320 */
        }

        /* Give initial trigger for the next frame */
        retVal = DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle,
                                       DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
        if(retVal < 0)
        {
            CLI_write("Error: DPU_RangeProcHWA_control failed with error code %d", retVal);
            DebugP_assert(0);
        }

        SemaphoreP_post(&gMmwMssMCB.tlvSemHandle);
        
    }
}

/**
*  @b Description
*  @n
*        Function configuring and executing DPC
*/
void MmwDemo_dpcTask()
{
    /* Save/restore FP registers during the context switching */
    vPortTaskUsesFPU();
    
    DPC_Config();

    DPC_Execute();

    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.TestSemHandle, SystemP_WAIT_FOREVER);
}



void DPC_mss_MsgHandler(uint32_t remoteCoreId, uint16_t localClientId, uint64_t msgValue, int32_t crcStatus, void *arg)
{
    uint32_t message;
    uint32_t messageArg;


    message     = (uint32_t) ((msgValue >> 32) & 0xffff);
    messageArg  = (uint32_t) (msgValue  & 0xffffffff);

    switch (message)
    {
        case DPC_DSS_TO_MSS_CONFIGURATION_COMPLETED:
            /* Send signal to CLI task that this is ready */
            SemaphoreP_post(&gMmwMssMCB.dspCfgDoneSemaphore);
            break;

        case DPC_DSS_TO_MSS_POINT_CLOUD_READY:

            /* Get the pointer to point cloud result from DSP */
            gMmwMssMCB.outputFromDSP = (DPIF_MSS_DSS_radarProcessOutput  *) messageArg;
            /* Send signal to classifier task this is ready */
            SemaphoreP_post(&gMmwMssMCB.classifierTaskSemHandle);
            break;
    }
}


