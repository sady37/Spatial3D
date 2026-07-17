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

#ifndef FACTORY_CAL_H
#define FACTORY_CAL_H

#define RF_SYNTH_TRIM_VALID        (1U)

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibData_t
{
    /*! @brief      Magic word for calibration data */
    uint32_t        magic;

    /*! @brief      RX TX Calibration data */
    T_RL_API_FECSS_FACT_CAL_DATA  calibData;

} MmwDemo_calibData;

/*This function reads calibration data from flash and send it to front end*/
static int32_t MmwDemo_calibRestore(MmwDemo_calibData  *ptrCalibData);
/*This function retrieves the calibration data from front end and saves it in flash.*/
static int32_t MmwDemo_calibSave(MmwDemo_calibData  *ptrCalibrationData);
/*This function performs factory calibration and saves it in flash*/
int32_t mmwDemo_factoryCal(void);

#endif /* FACTORY_CAL_H */
