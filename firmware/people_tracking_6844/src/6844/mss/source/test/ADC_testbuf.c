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
#include <utils/mathutils/mathutils.h>

#include <source/mmwave_demo_mss.h>
#include <source/test/ADC_testbuf.h>
#include <source/dpc/dpc_mss.h>

#define MAX_NUM_TX_ANTENNA          (SYS_COMMON_NUM_TX_ANTENNAS)
#define MAX_NUM_RX_ANTENNA          (SYS_COMMON_NUM_RX_CHANNEL)
#define MAX_AZ_FFT_SIZE             (64U)
#define MAX_NUM_RANGEBIN            (64U)
#define MAX_NUM_CHIRPS_PERFRAME     (128U)
#define READ_LINE_BUFSIZE   256
//Uncomment this for Low power mode verification - bit-matching with uninterrupted power mode
//#define LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION 

extern MmwDemo_MSS_MCB gMmwMssMCB;

int16_t *gPreStoredAdcTestBuff;
int32_t gPreStoredAdcTestBuffInd = 0;
int32_t gPreStoredAdcTestBuffRdInd = 0;

typedef struct rangeProcTestConfig_t_ {
    uint32_t numTxAntennas;
    uint32_t numRxAntennas;
    uint32_t numVirtualAntennas;
    uint32_t numAdcSamples;
    uint32_t numRangeBins;
    uint32_t numChirpsPerFrame;
    uint32_t numChirpsPerFrameRef;
    uint32_t numFrames;
} rangeProcTestConfig_t;

uint32_t  localRead(uint8_t * adcTestBuff, uint32_t sizeOfSamp,  uint32_t numSamp)
{
    memcpy((uint8_t *)adcTestBuff, (uint8_t *)&gPreStoredAdcTestBuff[gPreStoredAdcTestBuffInd], sizeOfSamp * numSamp);
    gPreStoredAdcTestBuffInd += numSamp;
    return numSamp;
}

#if defined(LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION)

uint32_t *gStoredHeatMap;
uint32_t gStoredHeatMapInd = 0;

void localWrite(uint32_t *detMatrixData, uint32_t sizeOfSamp, uint32_t numSamp, FILE *fileIdDetMatData)
{
    memcpy((uint8_t *)&gStoredHeatMap[gStoredHeatMapInd], (uint8_t *) detMatrixData, sizeOfSamp * numSamp);
    gStoredHeatMapInd += numSamp;
}
#endif

#if (CLI_REMOVAL == 0)

char *gAdcDataReadReady = "ready";
/**
*  @b Description
*  @n
*        Function writes "ready" message to host
*/
void MmwDemo_uartWriteAdcDataReady (UART_Handle handle)
{
    char *payload = gAdcDataReadReady;
    uint32_t payloadLength = strlen(payload);
    uint64_t timeout_usecs = 100000000;

    UART_Transaction trans;
    UART_Transaction_init(&trans);

    trans.buf   = (uint8_t *)payload;
    trans.count = payloadLength;
    trans.timeout =  ClockP_usecToTicks(timeout_usecs);

    UART_write(handle, &trans);
}

/**
*  @b Description
*  @n
*        Function reads ADC data from the host via UART port
*/
void MmwDemo_uartReadAdcData (UART_Handle handle,
                              uint8_t *payload,
                              uint32_t payloadLength)
{
    int32_t  transferOK;
    UART_Transaction trans;
    UART_Transaction_init(&trans);
    uint64_t timeout_usecs = 100000000;


    /* Read ADC data */
    trans.buf   = payload;
    trans.count = payloadLength;
    trans.timeout =  ClockP_usecToTicks(timeout_usecs);

    transferOK = UART_read(handle, &trans);
    if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
    {
        printf("Error reading ADC data \n");
        DebugP_assert(0);
    }
}


/**
*  @b Description
*  @n
*        Function to read ADC data from file. For testing purpose only.
*        When FeatureLiteBuild is enabled offline adc data injection cannot be used.
*/
void MmwDemo_adcFileReadTask(){
    UART_Handle uartHandle = gUartHandle[1];
    uint32_t baseAddr, regionId, numAdcSamplesPerEvt, numReadSamples;
    int32_t  errorCode = 0;
    int32_t status = SystemP_SUCCESS;
    uint16_t frameCnt, i;
    bool endOfFile = false;
    FILE * fileIdAdcData;
    //FILE * fileIdDetMatData;

    //FILE * fileIdDetMatDataRef;  //For CFAR offline verification

    //FILE * fileIdPointCloudIndData;
    rangeProcTestConfig_t testConfig;
    //char fileOutName[DPC_ADC_FILENAME_MAX_LEN];
    //char *ptr;
    uint8_t testCfgStr[READ_LINE_BUFSIZE];
    //uint32_t *detMatrixData;
#ifndef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
    //DPC_ObjectDetection_ExecuteResult *result = &gMmwMssMCB.dpcResult;
#endif
    
    //detMatrixData = (uint32_t *) gMmwMssMCB.detMatrix.data;

    /* Task is using floating point registers */
    vPortTaskUsesFPU();
    
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* start the test */
    if (gMmwMssMCB.adcDataSourceCfg.source == 1)
    {
        fileIdAdcData = fopen(gMmwMssMCB.adcDataSourceCfg.fileName, "rb");
        if (fileIdAdcData == NULL)
        {
            printf("Error:  Cannot open ADC file !\n");
            exit(0);
        }
/* For verifying CFAR*/
#if 0
    /* Open output file for detection matrix target */
    strcpy(fileOutName, gMmwMssMCB.adcDataSourceCfg.fileName);
    ptr = strrchr(fileOutName, '/');
    if (ptr == NULL)
    {
        strcpy(fileOutName, "detMatRef.bin");
    }
    else
    {
        strcpy(&ptr[1], "detMatRef.bin");
    }
    fileIdDetMatDataRef = fopen(fileOutName, "rb");
#endif


        /* Open output file for detection matrix target */
#if 0
        strcpy(fileOutName, gMmwMssMCB.adcDataSourceCfg.fileName);
        ptr = strrchr(fileOutName, '/');
        if (ptr == NULL)
        {
            strcpy(fileOutName, "detMatTarget.bin");
        }
        else
        {
            strcpy(&ptr[1], "detMatTarget.bin");
        }
        fileIdDetMatData = fopen(fileOutName, "wb");
        if (fileIdDetMatData == NULL)
        {
            printf("Error:  Cannot open Detection Matrix file !\n");
            exit(0);
        }

        /* Open output file for point cloud list: range/azimuth/elevation/Doppler indices  */
        strcpy(fileOutName, gMmwMssMCB.adcDataSourceCfg.fileName);
        ptr = strrchr(fileOutName, '/');
        if (ptr == NULL)
        {
            strcpy(fileOutName, "pcloudIndTarget.bin");
        }
        else
        {
            strcpy(&ptr[1], "pcloudIndTarget.bin");
        }
        fileIdPointCloudIndData = fopen(fileOutName, "wb");
        if (fileIdPointCloudIndData == NULL)
        {
            printf("Error:  Cannot open Point Cloud file !\n");
            exit(0);
        }
#endif
    }


    /* read number of frames in the file */
    if (gMmwMssMCB.adcDataSourceCfg.source == 1)
    {
        fread(&testConfig.numFrames, sizeof(uint32_t),1,fileIdAdcData);
    }
    else if (gMmwMssMCB.adcDataSourceCfg.source == 2)
    {
        ClockP_usleep(1000000);
        printf("First ready...");
        MmwDemo_uartWriteAdcDataReady(uartHandle);
        printf(" Sent\n");

        printf("read number of frames...");
        MmwDemo_uartReadAdcData (uartHandle,
                                  (uint8_t *)&testConfig.numFrames,
                                  sizeof(uint32_t));
        printf("Received: numFrames = %d\n", testConfig.numFrames);
    }

    else
    {
        memset ((void *)&testCfgStr[0], 0, sizeof(testCfgStr));
        status = CLI_readLine(gUartHandle[0], (char*)&testCfgStr[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            CLI_write("Error reading input config\n");
            DebugP_assert(0);
        }
        testConfig.numAdcSamples = atoi ((char*)&testCfgStr[0]);
        
        memset ((void *)&testCfgStr[0], 0, sizeof(testCfgStr));
        status = CLI_readLine(gUartHandle[0], (char*)&testCfgStr[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            CLI_write("Error reading input config\n");
            DebugP_assert(0);
        }
        testConfig.numVirtualAntennas = atoi ((char*)&testCfgStr[0]);
        
        memset ((void *)&testCfgStr[0], 0, sizeof(testCfgStr));
        status = CLI_readLine(gUartHandle[0], (char*)&testCfgStr[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            CLI_write("Error reading input config\n");
            DebugP_assert(0);
        }
        testConfig.numChirpsPerFrame = atoi ((char*)&testCfgStr[0]);

        memset ((void *)&testCfgStr[0], 0, sizeof(testCfgStr));
        status = CLI_readLine(gUartHandle[0], (char*)&testCfgStr[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            CLI_write("Error reading input config\n");
            DebugP_assert(0);
        }
        testConfig.numFrames = atoi ((char*)&testCfgStr[0]);

    }
    
    testConfig.numAdcSamples = gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples;
    testConfig.numTxAntennas = gMmwMssMCB.numTxAntennas;
    testConfig.numRxAntennas = gMmwMssMCB.numRxAntennas;
    testConfig.numVirtualAntennas = gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas;
    testConfig.numChirpsPerFrameRef = gMmwMssMCB.sigProcChainCommonCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst;
    testConfig.numChirpsPerFrame = testConfig.numChirpsPerFrameRef;
    testConfig.numRangeBins = gMmwMssMCB.numRangeBins;

    numAdcSamplesPerEvt = (testConfig.numAdcSamples * testConfig.numRxAntennas);

    if ((testConfig.numTxAntennas > MAX_NUM_TX_ANTENNA) || (testConfig.numRangeBins > MAX_NUM_RANGEBIN) || (testConfig.numChirpsPerFrame > MAX_NUM_CHIRPS_PERFRAME))
    {
        CLI_write("Error: Wrong test configurations \n");
        exit(0);
    }

    if ((testConfig.numFrames > 0) && (testConfig.numFrames < 65536))
    {
        //ToDo check that 4 params from ADC file match CLI configuration
        DebugP_log("numTxAntennas = %d\r", testConfig.numTxAntennas);
        DebugP_log("numRangeBins = %d\r", testConfig.numRangeBins);
        DebugP_log("numChirpsPerFrame = %d\n", testConfig.numChirpsPerFrame);
        DebugP_log("numFrames = %d\n", testConfig.numFrames);
    }
    else
    {
        CLI_write("Error: Wrong test configurations \n");
        DebugP_log("numFrames = %d\n", testConfig.numFrames);
        exit(0);
    }

#if 0
#ifdef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
    testConfig.numFrames = 8; //Do only 8 frames
    gPreStoredAdcTestBuff  = (int16_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                 testConfig.numAdcSamples *
                                                                 testConfig.numRxAntennas *
                                                                 testConfig.numChirpsPerFrame *
                                                                 testConfig.numTxAntennas *
                                                                 testConfig.numFrames *
                                                                 sizeof(uint16_t),
                                                                 sizeof(uint16_t));
    gStoredHeatMap   = (uint32_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                           testConfig.numRangeBins *
                                                           gMmwMssMCB.sigProcChainCfg.azimuthFftSize *
                                                           testConfig.numFrames *
                                                           sizeof(uint32_t),
                                                           sizeof(uint32_t));
    numReadSamples = testConfig.numAdcSamples * testConfig.numRxAntennas * testConfig.numChirpsPerFrame * testConfig.numTxAntennas * testConfig.numFrames * sizeof(uint16_t);

    if (gPreStoredAdcTestBuff == NULL)
    {
        printf("Error: Memory not allocated to store test ADC data !\n");
        exit(0);
    }
    else
    {
        //Read numFrames into buffer:
        numReadSamples = fread(gPreStoredAdcTestBuff, sizeof(uint16_t), numReadSamples, fileIdAdcData);
    }

#else
    if (gMmwMssMCB.adcDataSourceCfg.source == 2)
    {
        char localAdcTestBuff[READ_LINE_BUFSIZE];
        testConfig.numFrames = gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames; 
        if((testConfig.numFrames == 0) || (testConfig.numFrames > 8)) //Do a max of 8 frames
        {
            testConfig.numFrames = 8;
        }
        numReadSamples = testConfig.numAdcSamples * testConfig.numRxAntennas * testConfig.numChirpsPerFrame * testConfig.numTxAntennas * testConfig.numFrames * sizeof(uint16_t);

        gPreStoredAdcTestBuff  = (int16_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                    testConfig.numAdcSamples *
                                                                    testConfig.numRxAntennas *
                                                                    testConfig.numChirpsPerFrame *
                                                                    testConfig.numTxAntennas *
                                                                    testConfig.numFrames *
                                                                    sizeof(uint16_t),
                                                                    sizeof(uint16_t));
        
        if (gPreStoredAdcTestBuff == NULL)
        {
            printf("Error: Memory not allocated to store test ADC data !\n");
            exit(0);
        }

        
        //Read numFrames into buffer:
        //CLI_write("\n\nReading adc samples for all chirps and frames\n\n");
        for(frameCnt = 0; frameCnt < testConfig.numFrames; frameCnt++)
        {
            /* Read chirps from the file */
            for(i = 0; i < (testConfig.numChirpsPerFrame); i++)
            {
                for (j = 0; j < numAdcSamplesPerEvt; j++)
                {
                    status = CLI_readLine(gUartHandle[0], (char*)localAdcTestBuff, READ_LINE_BUFSIZE);
                    if(status != SystemP_SUCCESS)
                    {
                        CLI_write("Error reading input data\n");
                        DebugP_assert(0);
                    }
                    gPreStoredAdcTestBuff[gPreStoredAdcTestBuffRdInd] = (int16_t) atoi ((char*)localAdcTestBuff);
                    gPreStoredAdcTestBuffRdInd++;
                }
            }
        }
    }
#endif
#endif

    for(frameCnt = 0; frameCnt < testConfig.numFrames; frameCnt++)
    {

        gMmwMssMCB.stats.frameStartIntCounter++;
        printf("Start frame number %d\n", frameCnt);
        /* Read chirps from the file */
        for(i = 0; i < (testConfig.numChirpsPerFrame); i++)
        {
            if ((!endOfFile) && (gMmwMssMCB.adcDataSourceCfg.source != 0))
            {
                /* Read one chirp of ADC samples and to put data in ADC test buffer */
#ifndef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
                if (gMmwMssMCB.adcDataSourceCfg.source == 1)
                {
                    numReadSamples = fread(gMmwMssMCB.adcTestBuff, sizeof(uint16_t),  numAdcSamplesPerEvt, fileIdAdcData);
                }
                else if (gMmwMssMCB.adcDataSourceCfg.source == 2)
                {
                    //printf("R");
                    MmwDemo_uartWriteAdcDataReady(uartHandle);
                    //printf("S");
                    MmwDemo_uartReadAdcData (uartHandle,
                                              (uint8_t *)gMmwMssMCB.adcTestBuff,
                                              numAdcSamplesPerEvt * sizeof(uint16_t));
                    //printf("-");
                    numReadSamples = numAdcSamplesPerEvt;
                }
                else
                {
                    numReadSamples = localRead(gMmwMssMCB.adcTestBuff, sizeof(uint16_t),  numAdcSamplesPerEvt);
                }   

#else
                numReadSamples = localRead(gMmwMssMCB.adcTestBuff, sizeof(uint16_t),  numAdcSamplesPerEvt);
#endif
                if (numReadSamples != numAdcSamplesPerEvt)
                {
                    endOfFile = true;
                }
            }
            //printf("Chirp number %d read\n", i);
            /* Manual trigger to simulate chirp avail irq */
            errorCode = EDMAEnableTransferRegion(
                            baseAddr, regionId, EDMA_DSS_TPCC_A_CHIRP_AVAIL_IRQ, EDMA_TRIG_MODE_MANUAL); //EDMA_TRIG_MODE_EVENT
            if (errorCode != 1)
            {
                CLI_write("Error: EDMA start Transfer returned %d\n",errorCode);
                return;
            }

            if (gMmwMssMCB.adcDataSourceCfg.source == 1)
            {
            // ToDo return this back, and verify, since the potentiial fix was applied  
            //    ClockP_usleep(1000); //1ms sleep   Note: comented out since the loop was stuck randomly afte some number of chirp iterations
            }

        } /* end of chirp loop */
        //printf("\n");
        SemaphoreP_pend(&gMmwMssMCB.adcFileTaskSemHandle, SystemP_WAIT_FOREVER);

#ifndef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
#if 0
        if (gMmwMssMCB.adcDataSourceCfg.source == 1)
        {
            /* Write out Detection Matrix */
            fwrite(detMatrixData, sizeof(uint32_t), gMmwMssMCB.intrusionSigProcChainCfg.azimuthFftSize * testConfig.numRangeBins, fileIdDetMatData);

            //Write out point cloud
            fwrite(&frameCnt, sizeof(uint16_t), 1, fileIdPointCloudIndData);
            fwrite(&result->numObjOut, sizeof(uint16_t), 1, fileIdPointCloudIndData);
            if(result->numObjOut > 0)
            {
                int objInd;
                for (objInd = 0; objInd < result->numObjOut; objInd++)
                {
                    fwrite(&gMmwMssMCB.dpcObjIndOut[objInd], sizeof(DPIF_PointCloudRngAzimElevDopInd), 1, fileIdPointCloudIndData);
                    fwrite(&gMmwMssMCB.cfarDetObjOut[objInd].snr, sizeof(uint16_t), 1, fileIdPointCloudIndData);
                    fwrite(&gMmwMssMCB.cfarDetObjOut[objInd].noise, sizeof(uint16_t), 1, fileIdPointCloudIndData);
                }
            }
        }
#endif
        printf("ADC file read task: Processed frame number %d\n", frameCnt);
#else
        if (gMmwMssMCB.adcDataSourceCfg.source == 1)
        {
            /* Write out Detection Matrix to L3 memory*/
            localWrite(detMatrixData, sizeof(uint32_t), gMmwMssMCB.sigProcChainCfg.azimuthFftSize * testConfig.numRangeBins, fileIdDetMatData);
        }
#endif


    } /* end of frame loop */

    if (gMmwMssMCB.adcDataSourceCfg.source == 1)
    {
        fclose(fileIdAdcData);
#if 0
        fclose(fileIdDetMatData);
        fclose(fileIdPointCloudIndData);
#endif
    }

    /* check the result */
    DebugP_log("Test finished!\n\r");
    DebugP_log("\n... DPC Finished, Check Output data ....  : \n\n");

}
#endif
