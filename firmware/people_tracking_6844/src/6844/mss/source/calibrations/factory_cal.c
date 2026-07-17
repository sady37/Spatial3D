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
#include <string.h>
#include <stdio.h>

#include <source/mmwave_demo_mss.h>
#include <source/calibrations/mmw_flash_cal.h>
#include <source/mmw_res.h>
#include <source/mmw_cli.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>
#include <source/calibrations/factory_cal.h>

/* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_STORE_MAGIC            (0x7CB28DF9U)

extern MmwDemo_MSS_MCB gMmwMssMCB;

#if (ENABLE_MONITORS==1)
/*! @brief  RF Monitor LB result during factory calibration */
volatile MmwDemo_Mon_Result rfMonResFactCal = {0};
#endif

MmwDemo_calibData gFactoryCalibDataStorage __attribute__((aligned(8))) = {0};

/**
 *  @b Description
 *  @n
 *      This function reads calibration data from flash and send it to front end
 *
 *  @param[in]  ptrCalibData         Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibRestore(MmwDemo_calibData  *ptrCalibData)
{
    int32_t    retVal = 0;
    uint32_t   flashOffset;

    /* Get Flash Offset */
    flashOffset = gMmwMssMCB.mmWaveCfg.calibCfg.flashOffset;

    /* Read calibration data */
    if(MmwDemo_flashRead(flashOffset, (uint8_t *)ptrCalibData, sizeof(MmwDemo_calibData) )< 0)
    {
        /* Error: Failed to read from Flash */
        CLI_write ("Error: MmwDemo failed when reading Factory calibration data from flash.\r\n");
        return -1;
    }

    /* Validate Calib data Magic number */
    if(ptrCalibData->magic != MMWDEMO_CALIB_STORE_MAGIC)
    {
        /* Header validation failed */
        CLI_write ("Error: MmwDemo Factory calibration data header validation failed.\r\n");
        return -1;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function retrieves the calibration data from front end and saves it in flash.
 *
 *  @param[in]  ptrCalibrationData      Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibSave(MmwDemo_calibData  *ptrCalibrationData)
{
    uint32_t                flashOffset;
    int32_t                 retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.mmWaveCfg.calibCfg.flashOffset;

    /* Flash calibration data */
    retVal = MmwDemo_flashWrite(flashOffset, (uint8_t *)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if(retVal < 0)
    {
        /* Flash write failed */
        CLI_write ("Error: MmwDemo failed flashing calibration data with error[%d].\n", retVal);
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function performs factory calibration and saves it in flash
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
/* Note: In realtime applications, factory calibration is a one-time activity and users are expected to perform this only once */
int32_t mmwDemo_factoryCal(void)
{
    int32_t          retVal = SystemP_SUCCESS;
    int32_t          errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t          mmWaveErrorCode;
    int16_t          subsysErrorCode;

    /* Initialize Factory Calib Data Pointer to NULL */
    gMmwMssMCB.mmWaveCfg.calibCfg.ptrFactoryCalibData = NULL;

    /* If restore option is selected, Factory Calibration is not re-run and data is restored from Flash */ 
    if(gMmwMssMCB.mmWaveCfg.calibCfg.restoreEnable == 1U)
    {
        if(MmwDemo_calibRestore(&gFactoryCalibDataStorage) < 0)
        {
            CLI_write ("Error: MmwDemo failed restoring Factory Calibration data from flash.\r\n");
            MmwDemo_debugAssert (0);
        }

        /* Populate calibration data pointer. */
        gMmwMssMCB.mmWaveCfg.calibCfg.ptrFactoryCalibData = &gFactoryCalibDataStorage.calibData;
    }

    if(gMmwMssMCB.mmWaveCfg.calibCfg.saveEnable == 1U)
    {
        gFactoryCalibDataStorage.magic = MMWDEMO_CALIB_STORE_MAGIC;
        gMmwMssMCB.mmWaveCfg.calibCfg.ptrFactoryCalibData = &gFactoryCalibDataStorage.calibData;
    }

    retVal = MMWave_factoryCalib(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmWaveCfg, &errCode);
    if (retVal != SystemP_SUCCESS)
    {

        /* Error: Unable to perform boot calibration */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Error: Unable to initialize the mmWave control module */
        CLI_write ("Error: mmWave Control Initialization failed [Error code %d] [errorLevel %d] [mmWaveErrorCode %d] [subsysErrorCode %d]\n", errCode, errorLevel, mmWaveErrorCode, subsysErrorCode);
        if (mmWaveErrorCode == MMWAVE_ERFSBOOTCAL)
        {
            CLI_write ("Error: Factory Calibration failure\n");
        }
        else
        {
            CLI_write ("Error: Invalid Factory calibration arguments\n");
            MmwDemo_debugAssert (0);
        }
    }

    /* Save calibration data in flash */
    if(gMmwMssMCB.mmWaveCfg.calibCfg.saveEnable != 0)
    {
#if 0 //ToDo: enable this after the MmwDemo_calibSave is fixed 
            /* Save data in flash */
            retVal = MmwDemo_calibSave(&gFactoryCalibDataStorage);
            if(retVal < 0)
            {
                CLI_write("Error: MMW demo failed Calibration Save with Error[%d]\n", retVal);
                MmwDemo_debugAssert (0);
            }
#endif
    }
    return retVal;
}
