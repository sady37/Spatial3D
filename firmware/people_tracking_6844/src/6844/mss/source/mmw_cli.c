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

#include <source/mmw_cli.h>
#include <drivers/uart.h>
#include <drivers/prcm.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/csl_complex_math_types.h>
#include <utils/mathutils/mathutils.h>
#include <common/syscommon.h>
#include <datapath/dpif/dpif_adcdata.h>
#include <drivers/gpio.h>
#include <drivers/edma.h>
#include <mmwavelink/include/rl_device.h>

#include "tracker_utils/tracker_utils.h"

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"

#include <source/calibrations/factory_cal.h>

#define CLI_TASK_STACK_SIZE  (4 *1024U)
#define READ_LINE_BUFSIZE   2048

#define PI 3.14159265358979323846f
#define RAD2DEG (180.f/PI)
#define DEG2RAD (PI/180.f)


/* Demo Flash Address offset on 1MB */
#define MMWDEMO_CALIB_FLASH_ADDR_1MB  (uint32_t)(0x100000U)

//Enable this define, to allow dynamic swith to 1.25Mbps baud rate, then execute CLI cmd: baudRate 1250000
#define ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG

/* Device string used to print in the CLI banner */
#if defined (SOC_XWRL684X)
#define DEVICE_STRING "xWRL684x"
#endif

uint32_t gGpioBaseAddrLed;
uint32_t gPinNumLed;

extern uint32_t gSPIHostIntrBaseAddrLed,gSPIHostIntrPinNumLed;
// Indicate if Sensor is Started or not
uint8_t gIsSensorStarted = 0;

/* Variable to check if antenna geometry is defined: 1000b indicates its fully defined */
static uint8_t GIsAntGeoDef = 1;
/* Variable to check if range bias and phase compensation is defined: 0: Not defined 1: fully defined */
static uint8_t GIsRangePhaseCompDef = 0;

/* Arrays to store the TX and RX antenna indices */
int8_t GAntGeometryTX[2*SYS_COMMON_NUM_TX_ANTENNAS];
int8_t GAntGeometryRX[2*SYS_COMMON_NUM_RX_CHANNEL];

// /* CLI commands designated for antenna geometry, range and phase compensation, these are specific to the xWRL6844EVM. For other devices, ensure a different global variable is used accordingly */
// char* GantGeoRangePhaseCompxWRL6844EVM[CLI_ANT_GEO_PHASE_COMP_CMD] = {
//     "antGeometryTX 2 2 2 0 0 0 0 2 \r\n",
//     "antGeometryRX 1 0 0 0 0 1 1 1 \r\n",
//     "antGeometryDist 2.54 2.54 \r\n"
// };

/* CLI commands designated for antenna geometry, range and phase compensation, these are specific to the xWRL6844EVM. For other devices, ensure a different global variable is used accordingly */
char* GantGeoRangePhaseCompxWRL6844EVM[CLI_ANT_GEO_PHASE_COMP_CMD] = {
    "antGeometryTX 2 2 2 0 0 0 0 2 \r\n",
    "antGeometryRX 1 0 0 0 0 1 1 1 \r\n",
    "antGeometryDist 2.54 2.54 \r\n",
    "compRangeBiasAndRxChanPhase 0.0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 \r\n"
};

TaskHandle_t gCliTask;
StaticTask_t gCliTaskObj;
StackType_t  gCliTskStack[CLI_TASK_STACK_SIZE] __attribute__((aligned(32)));

uint16_t gTxAntMask[SYS_COMMON_NUM_TX_ANTENNAS] = {0x03, 0x0C, 0x30, 0xC0};
uint16_t gRxAntMask[SYS_COMMON_NUM_RX_CHANNEL] = {0x01, 0x08, 0x10, 0x80};

static SemaphoreP_Object gUartReadDoneSem;

CLI_MCB     gCLI;
#if (CLI_BYPASS == 1)
/* When CLI-BYPASS is enabled, CLI configurations specified in this structure are used */
//Original cpd
#if 1
char* GRadarCmdString[] =
{
 "sensorStop 0 \n\r",
 "channelCfg 15 15 0  \n\r",
 "chirpComnCfg 50 0 0 128 1 37 1  \n\r",
 "chirpTimingCfg 4 28 1.5 105 57.5  \n\r",
 "frameCfg 4 12 4000 32 200 0 \n\r",
 "runningMode 2 1 \n\r",
 "sigProcChainCommonCfg 4 32 200 0 0 \n\r",
 "macroDopplerCfg 1 0 0 40 128 32 0 1 \n\r",
 "macroDopSteerDbgCfg 0 \n\r",
 "guiMonitor 1 0 1 0 0 1 1 \n\r",
 "dbgGuiMonitor 0 0 0 0  0 0 0 0   0 1 \n\r",
 "exportRadCubeChunk 0 25 1 \n\r",
 "dynamicRACfarCfg   5 15 1 1 8 8 4 6 4 1 8.00 6.00 0.50 1 15 \n\r",
 "staticRACfarCfg 4 4 2 2 8 16 4 6 6.00 13.00 0.50 0  \n\r",
 "dynamicRangeAngleCfg 8.000 0.03 2 0 \n\r",
 "dynamic2DAngleCfg 5 1 1 1.00 10.00 2 \n\r",
 "staticRangeAngleCfg 0 1 1 \n\r",
 "dopplerBinSelCfg 1 32 0 4 \n\r",
 "antGeometry0 -2 -2 -3 -3   0  0 -1 -1    0  0 -1 -1   -2 -2 -3 -3 \n\r",
 "antGeometry1  0 -1 -1  0   0 -1 -1  0   -2 -3 -3 -2   -2 -3 -3 -2 \n\r",
 "antPhaseRot 1 1 1 1  1 1 1 1  1 1 1 1  1 1 1 1  \n\r",
 "compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0  \n\r",
 "fovCfg 75.0 75.0 \n\r",
 "sensorPosition 0 0.8 1.1 0 -60 \n\r",
 "cuboidDef 0 0   0.1 0.75    0.5 1.30   0.85  1.3 \n\r",
 "cuboidDef 0 1   0.1 0.75    0.5 1.30    0.3  0.85 \n\r",
 "cuboidDef 0 2   0.1 0.75    0.1 0.9   0  0.4 \n\r",
 "cuboidDef 1 0  -0.75 -0.1   0.5 1.30   0.85  1.3 \n\r",
 "cuboidDef 1 1  -0.75 -0.1  0.5 1.30    0.3  0.85 \n\r",
 "cuboidDef 1 2  -0.75 -0.1  0.1 0.9   0  0.4 \n\r",
 "cuboidDef 2 0   0.2 0.75    1.4 2.2    0.85  1.3 \n\r",
 "cuboidDef 2 1   0.2 0.75    1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 2 2   0.2 0.75    1.3 1.6    -0.2  0.4 \n\r",
 "cuboidDef 3 0  -0.12 0.12    1.4 2.2    0.85  1.3 \n\r",
 "cuboidDef 3 1  -0.12 0.12    1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 3 2  -0.12 0.12    1.3 1.6    0  0.4 \n\r",
 "cuboidDef 4 0  -0.75 -0.2   1.4 2.2   0.85  1.3 \n\r",
 "cuboidDef 4 1  -0.75 -0.2   1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 4 2  -0.75 -0.2   1.3 1.6    -0.2  0.4 \n\r",
 "featExtrCfg 130 30 1 0 0.4 10 \n\r",
 "zOffsetCfg 0.01 -0.06 0.05 0 -0.08 \n\r",
 "macroDopMapScaleCfg 1.0 1.0 1.0 1.0 1.0 \n\r",
 "macroDopNumVoxelCfg 30 30 30 30 30 \n\r",
 "macroDopRngBinOffsetCfg 5 5 20 14 20 \n\r",
 "factoryCalibCfg 1 0 46 0 0x1ff000 \n\r",
 "runtimeCalibCfg 0 \n\r",
 "adcDataSource 0 adc_test_data_iwr6844.bin \n\r",
 "baudRate 6250000 \n\r",
 "sensorStart 0 0 0 0 \n\r",
 ""
};
#endif
//cpd_cispr_9_acc
#if 0
char* GRadarCmdString[] =
{
 "sensorStop 0 \n\r",
 "channelCfg 15 15 0 \n\r",
 "chirpComnCfg 50 0 0 128 1 38.25 1 \n\r",
 "chirpTimingCfg 4 28 1.5 102 57.5 \n\r",
 "frameCfg 4 9 1525 1 4.125 32 \n\r",
 "runningMode 2 1 \n\r",
 "sigProcChainCommonCfg 4 32 200 0 \n\r",
 "macroDopplerCfg 1 0 0 40 128 32 0 1 \n\r",
 "macroDopSteerDbgCfg 0 \n\r",
 "guiMonitor 1 0 1 0 0 1 1 \n\r",
 "dbgGuiMonitor 0 0 0 0  0 0 0 0   0 1 \n\r",
 "exportRadCubeChunk 0 25 1 \n\r",
 "dynamicRACfarCfg   5 15 1 1 8 8 4 6 4 1 8.00 6.00 0.50 1 15 \n\r",
 "staticRACfarCfg 4 4 2 2 8 16 4 6 6.00 13.00 0.50 0  \n\r",
 "dynamicRangeAngleCfg 8.000 0.03 2 0 \n\r",
 "dynamic2DAngleCfg 5 1 1 1.00 10.00 2 \n\r",
 "staticRangeAngleCfg 0 1 1 \n\r",
 "dopplerBinSelCfg 1 32 0 4 \n\r",
 "antGeometry0 -2 -2 -3 -3   0  0 -1 -1    0  0 -1 -1   -2 -2 -3 -3 \n\r",
 "antGeometry1  0 -1 -1  0   0 -1 -1  0   -2 -3 -3 -2   -2 -3 -3 -2 \n\r",
 "antPhaseRot 1 1 1 1  1 1 1 1  1 1 1 1  1 1 1 1  \n\r",
 "compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0  \n\r",
 "fovCfg 75.0 75.0 \n\r",
 "sensorPosition 0 0.8 1.1 0 -60 \n\r",
 "cuboidDef 0 0   0.1 0.75    0.5 1.30   0.85  1.3 \n\r",
 "cuboidDef 0 1   0.1 0.75    0.5 1.30    0.3  0.85 \n\r",
 "cuboidDef 0 2   0.1 0.75    0.1 0.9   0  0.4 \n\r",
 "cuboidDef 1 0  -0.75 -0.1   0.5 1.30   0.85  1.3 \n\r",
 "cuboidDef 1 1  -0.75 -0.1  0.5 1.30    0.3  0.85 \n\r",
 "cuboidDef 1 2  -0.75 -0.1  0.1 0.9   0  0.4 \n\r",
 "cuboidDef 2 0   0.2 0.75    1.4 2.2    0.85  1.3 \n\r",
 "cuboidDef 2 1   0.2 0.75    1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 2 2   0.2 0.75    1.3 1.6    -0.2  0.4 \n\r",
 "cuboidDef 3 0  -0.12 0.12    1.4 2.2    0.85  1.3 \n\r",
 "cuboidDef 3 1  -0.12 0.12    1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 3 2  -0.12 0.12    1.3 1.6    0  0.4 \n\r",
 "cuboidDef 4 0  -0.75 -0.2   1.4 2.2   0.85  1.3 \n\r",
 "cuboidDef 4 1  -0.75 -0.2   1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 4 2  -0.75 -0.2   1.3 1.6    -0.2  0.4 \n\r",
 "featExtrCfg 130 30 1 0 0.4 10 \n\r",
 "zOffsetCfg 0.01 -0.06 0.05 0 -0.08 \n\r",
 "macroDopMapScaleCfg 1.0 1.0 1.0 1.0 1.0 \n\r",
 "macroDopNumVoxelCfg 30 30 30 30 30 \n\r",
 "macroDopRngBinOffsetCfg 5 5 20 14 20 \n\r",
 "factoryCalibCfg 1 0 46 0 0x1ff000 \n\r",
 "runtimeCalibCfg 0 \n\r",
 "adcDataSource 0 adc_test_data_iwr6844.bin \n\r",
 "baudRate 6250000 \n\r",
 "sensorStart 0 0 0 0 \n\r",
 ""
};
#endif
//cpd_cispr_12_acc
#if 0
char* GRadarCmdString[] =
{
 "sensorStop 0 \n\r",
 "channelCfg 15 15 0 \n\r",
 "chirpComnCfg 50 0 0 128 1 38.25 1 \n\r",
 "chirpTimingCfg 4 28 1.5 102 57.5 \n\r",
 "frameCfg 4 12 2032 1 4.125 32 \n\r",
 "runningMode 2 1 \n\r",
 "sigProcChainCommonCfg 4 32 200 0 \n\r",
 "macroDopplerCfg 1 0 0 40 128 32 0 1 \n\r",
 "macroDopSteerDbgCfg 0 \n\r",
 "guiMonitor 1 0 1 0 0 1 1 \n\r",
 "dbgGuiMonitor 0 0 0 0  0 0 0 0   0 1 \n\r",
 "exportRadCubeChunk 0 25 1 \n\r",
 "dynamicRACfarCfg   5 15 1 1 8 8 4 6 4 1 8.00 6.00 0.50 1 15 \n\r",
 "staticRACfarCfg 4 4 2 2 8 16 4 6 6.00 13.00 0.50 0  \n\r",
 "dynamicRangeAngleCfg 8.000 0.03 2 0 \n\r",
 "dynamic2DAngleCfg 5 1 1 1.00 10.00 2 \n\r",
 "staticRangeAngleCfg 0 1 1 \n\r",
 "dopplerBinSelCfg 1 32 0 4 \n\r",
 "antGeometry0 -2 -2 -3 -3   0  0 -1 -1    0  0 -1 -1   -2 -2 -3 -3 \n\r",
 "antGeometry1  0 -1 -1  0   0 -1 -1  0   -2 -3 -3 -2   -2 -3 -3 -2 \n\r",
 "antPhaseRot 1 1 1 1  1 1 1 1  1 1 1 1  1 1 1 1  \n\r",
 "compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0  \n\r",
 "fovCfg 75.0 75.0 \n\r",
 "sensorPosition 0 0.8 1.1 0 -60 \n\r",
 "cuboidDef 0 0   0.1 0.75    0.5 1.30   0.85  1.3 \n\r",
 "cuboidDef 0 1   0.1 0.75    0.5 1.30    0.3  0.85 \n\r",
 "cuboidDef 0 2   0.1 0.75    0.1 0.9   0  0.4 \n\r",
 "cuboidDef 1 0  -0.75 -0.1   0.5 1.30   0.85  1.3 \n\r",
 "cuboidDef 1 1  -0.75 -0.1  0.5 1.30    0.3  0.85 \n\r",
 "cuboidDef 1 2  -0.75 -0.1  0.1 0.9   0  0.4 \n\r",
 "cuboidDef 2 0   0.2 0.75    1.4 2.2    0.85  1.3 \n\r",
 "cuboidDef 2 1   0.2 0.75    1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 2 2   0.2 0.75    1.3 1.6    -0.2  0.4 \n\r",
 "cuboidDef 3 0  -0.12 0.12    1.4 2.2    0.85  1.3 \n\r",
 "cuboidDef 3 1  -0.12 0.12    1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 3 2  -0.12 0.12    1.3 1.6    0  0.4 \n\r",
 "cuboidDef 4 0  -0.75 -0.2   1.4 2.2   0.85  1.3 \n\r",
 "cuboidDef 4 1  -0.75 -0.2   1.4 2.2    0.1  0.85 \n\r",
 "cuboidDef 4 2  -0.75 -0.2   1.3 1.6    -0.2  0.4 \n\r",
 "featExtrCfg 130 30 1 0 0.4 10 \n\r",
 "zOffsetCfg 0.01 -0.06 0.05 0 -0.08 \n\r",
 "macroDopMapScaleCfg 1.0 1.0 1.0 1.0 1.0 \n\r",
 "macroDopNumVoxelCfg 30 30 30 30 30 \n\r",
 "macroDopRngBinOffsetCfg 5 5 20 14 20 \n\r",
 "factoryCalibCfg 1 0 46 0 0x1ff000 \n\r",
 "runtimeCalibCfg 0 \n\r",
 "adcDataSource 0 adc_test_data_iwr6844.bin \n\r",
 "baudRate 6250000 \n\r",
 "sensorStart 0 0 0 0 \n\r",
 ""
};
#endif
static int32_t CLI_ByPassApi(CLI_Cfg* ptrCLICfg);
#endif

static int32_t CLI_help (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveExtensionHandler(int32_t argc, char* argv[]);
static void    CLI_MMWaveExtensionHelp(void);


static int32_t CLI_MMWaveVersion (int32_t argc, char* argv[]);

static int32_t CLI_MMWaveSensorStop (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChirpCommonCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveChirpTimingCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[]);

static int32_t CLI_MMWaveClutterRemoval (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveLowPwrModeEnable(int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFactoryCalConfig (int32_t argc, char* argv[]);
static int32_t CLI_MmwDemo_AntGeometryCfg (int32_t argc, char* argv[]);
static int32_t CLI_MmwDemo_AntGeometryBoard (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdcDataSourceCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdcLogging (int32_t argc, char* argv[]);
int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveRuntimeCalConfig (int32_t argc, char* argv[]);

// For  Minor Motion - Capon
static int32_t mmwLab_CLIDynRACfarCfg(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIStaticRACfarCfg(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIDynRngAngleCfg(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIStaticRngAngleCfg(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIDynAngleEstCfg(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIDoppBinSelCfg(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIDopplerCFARCfg(int32_t argc, char *argv[]);

static int32_t mmwLab_CLIBoardAntGeometry0(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIBoardAntGeometry1(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIBoardAntPhaseRot(int32_t argc, char *argv[]);
static int32_t mmwLab_CLIAntAngleFoV(int32_t argc, char *argv[]);



/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern uint32_t gSensorStop;
extern MmwDemo_MSS_MCB gMmwMssMCB;

extern void MmwDemo_startDemoProcessing(void);

/**
 * @brief
 *  This is the mmWave extension table added to the CLI.
 */
CLI_CmdTableEntry gCLIMMWaveExtensionTable[] =
{
    {
        "version",
        "No arguments",
        CLI_MMWaveVersion
    },
    {
        NULL,
        NULL,
        NULL
    }
};

static int32_t CLI_help (int32_t argc, char* argv[])
{
    uint32_t    index;

    /* Display the banner: */
    CLI_write ("Help: This will display the usage of the CLI commands\n");
    CLI_write ("Command: Help Description\n");

    /* Cycle through all the registered CLI commands: */
    for (index = 0; index < gCLI.numCLICommands; index++)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    gCLI.cfg.tableEntry[index].cmd,
                   (gCLI.cfg.tableEntry[index].helpString == NULL) ?
                    "No help available" :
                    gCLI.cfg.tableEntry[index].helpString);
    }

    /* Is the mmWave Extension enabled? */
    if (gCLI.cfg.enableMMWaveExtension == 1U)
    {
        /* YES: Pass the control to the extension help handler. */
        CLI_MMWaveExtensionHelp ();
    }
    return 0;
}

static int32_t CLI_MMWaveExtensionHandler(int32_t argc, char* argv[])
{
    CLI_CmdTableEntry*  ptrCLICommandEntry;
    int32_t             cliStatus;
    int32_t             retVal = 0;

    /* Get the pointer to the mmWave extension table */
    ptrCLICommandEntry = &gCLIMMWaveExtensionTable[0];

    /* Cycle through all the registered externsion CLI commands: */
    while (ptrCLICommandEntry->cmdHandlerFxn != NULL)
    {
        /* Do we have a match? */
        if (strcmp(ptrCLICommandEntry->cmd, argv[0]) == 0)
        {
            /* YES: Pass this to the CLI registered function */
            cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argc, argv);
            if (cliStatus == 0)
            {
                /* Successfully executed the CLI command: */
                CLI_write ("Done\r\n\n");
            }
            else
            {
                /* Error: The CLI command failed to execute */
                CLI_write ("Error %d\r\n", cliStatus);
            }
            break;
        }

        /* Get the next entry: */
        ptrCLICommandEntry++;
    }

    /* Was this a valid CLI command? */
    if (ptrCLICommandEntry->cmdHandlerFxn == NULL)
    {
        /* NO: The command was not a valid CLI mmWave extension command. Setup
         * the return value correctly. */
        retVal = -1;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void CLI_task(void* args)
{
    /* Save/restore FP registers during the context switching */
    vPortTaskUsesFPU();

    #if (CLI_BYPASS == 0)
    uint8_t                 cmdString[READ_LINE_BUFSIZE];
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus, status;
    uint32_t                index;


    /* Do we have a banner to be displayed? */
    if (gCLI.cfg.cliBanner != NULL)
    {
        /* YES: Display the banner */
        CLI_write (gCLI.cfg.cliBanner);
    }

    /* Loop around forever: */
    while (1)
    {
        /* Demo Prompt: */
        CLI_write (gCLI.cfg.cliPrompt);

        /* Reset the command string: */
        memset ((void *)&cmdString[0], 0, sizeof(cmdString));

        status = CLI_readLine(gCLI.cfg.UartHandle, (char*)&cmdString[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            CLI_write("Error reading\n");
        }

        /* Reset all the tokenized arguments: */
        memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
        argIndex      = 0;
        ptrCLICommand = (char*)&cmdString[0];

        /* comment lines found - ignore the whole line*/
        if (cmdString[0]=='%' || cmdString[1]=='%')
        {
            CLI_write ("Skipped\r\n");
            continue;
        }

        /* Set the CLI status: */
        cliStatus = -1;

        /* The command has been entered we now tokenize the command message */
        while (1)
        {
            /* Tokenize the arguments: */
            tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
            if (tokenizedArgs[argIndex] == NULL)
                break;

            /* Increment the argument index: */
            argIndex++;
            if (argIndex >= CLI_MAX_ARGS)
                break;

            /* Reset the command string */
            ptrCLICommand = NULL;
        }

        /* Were we able to tokenize the CLI command? */
        if (argIndex == 0)
            continue;

        /* Cycle through all the registered CLI commands: */
        for (index = 0; index < gCLI.numCLICommands; index++)
        {
            ptrCLICommandEntry = &gCLI.cfg.tableEntry[index];

            /* Do we have a match? */
            if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
            {
                /* YES: Pass this to the CLI registered function */
                cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
                if (cliStatus == 0)
                {
                    CLI_write ("Done\r\n\n");
                }
                else
                {
                    CLI_write ("Error %d\r\n", cliStatus);
                }
                break;
            }
        }

        /* Did we get a matching CLI command? */
        if (index == gCLI.numCLICommands)
        {
            /* NO matching command found. Is the mmWave extension enabled? */
            if (gCLI.cfg.enableMMWaveExtension == 1U)
            {
                /* Yes: Pass this to the mmWave extension handler */
                cliStatus = CLI_MMWaveExtensionHandler (argIndex, tokenizedArgs);
            }

            /* Was the CLI command found? */
            if (cliStatus == -1)
            {
                /* No: The command was still not found */
                CLI_write ("'%s' is not recognized as a CLI command\r\n", tokenizedArgs[0]);
            }
        }
    }
    #else

    CLI_ByPassApi(&gCLI.cfg);
    #endif
    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.cliInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);
}

static void CLI_MMWaveExtensionHelp(void)
{
    CLI_CmdTableEntry*  ptrCLICommandEntry;

    /* Get the pointer to the mmWave extension table */
    ptrCLICommandEntry = &gCLIMMWaveExtensionTable[0];

    /* Display the banner: */
    CLI_write ("****************************************************\n");
    CLI_write ("mmWave Extension Help\n");
    CLI_write ("****************************************************\n");

    /* Cycle through all the registered externsion CLI commands: */
    while (ptrCLICommandEntry->cmdHandlerFxn != NULL)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    ptrCLICommandEntry->cmd,
                   (ptrCLICommandEntry->helpString == NULL) ?
                    "No help available" :
                    ptrCLICommandEntry->helpString);

        /* Get the next entry: */
        ptrCLICommandEntry++;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the version command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  \ingroup CLI_UTIL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveVersion (int32_t argc, char* argv[])
{
    int32_t       retVal = 0;
    T_RL_API_DFP_FW_VER_GET_RSP dfpVerapiResData;

    if(gCLI.cfg.overridePlatform == false)
    {
#if defined (SOC_XWRL684X)
        /* print the platform */
        CLI_write ("Platform                : XWRL684x\r\n");
#endif
    }
    else
    {
        CLI_write ("Platform                : %s\r\n", gCLI.cfg.overridePlatformString);
    }

    /* Initialize API response Structures. */
    memset(&dfpVerapiResData,0,sizeof(T_RL_API_DFP_FW_VER_GET_RSP));

    /* Get the version string: */
    retVal = rl_mmWaveDfpVerGet(M_DFP_DEVICE_INDEX_0, &dfpVerapiResData);
    if (retVal < 0)
    {
        CLI_write ("Error: get DFP version [Error %d]\r\n", retVal);
        return -1;
    }

    CLI_write ("RFS Firmware Version    : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_RfsRomVersion.c_GenVerNum,
                dfpVerapiResData.z_RfsRomVersion.c_MajorVerNum,
                dfpVerapiResData.z_RfsRomVersion.c_MinorVerNum,
                dfpVerapiResData.z_RfsRomVersion.c_BuildVerNum);

    CLI_write ("FECSS Lib Version       : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_FecssLibVersion.c_GenVerNum,
                dfpVerapiResData.z_FecssLibVersion.c_MajorVerNum,
                dfpVerapiResData.z_FecssLibVersion.c_MinorVerNum,
                dfpVerapiResData.z_FecssLibVersion.c_BuildVerNum);

    CLI_write ("mmWaveLink Version      : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_MmwlLibVersion.c_GenVerNum,
                dfpVerapiResData.z_MmwlLibVersion.c_MajorVerNum,
                dfpVerapiResData.z_MmwlLibVersion.c_MinorVerNum,
                dfpVerapiResData.z_MmwlLibVersion.c_BuildVerNum);

    CLI_write ("RFS Patch Version       : %02d.%02d.%02d.%02d\r\n",
                dfpVerapiResData.z_RfsPatchVersion.c_GenVerNum,
                dfpVerapiResData.z_RfsPatchVersion.c_MajorVerNum,
                dfpVerapiResData.z_RfsPatchVersion.c_MinorVerNum,
                dfpVerapiResData.z_RfsPatchVersion.c_BuildVerNum);

    /* Display the version information on the CLI Console: */
    CLI_write ("mmWave SDK Version      : %02d.%02d.%02d.%02d\r\n",
                            MMWAVE_SDK_VERSION_MAJOR,
                            MMWAVE_SDK_VERSION_MINOR,
                            MMWAVE_SDK_VERSION_BUGFIX,
                            MMWAVE_SDK_VERSION_BUILD);
    /* Version string has been formatted successfully. */
    /* Display the Demo information on the CLI Console: */
    CLI_write ("MMWAVE DEMO for xwrL684x\r\n");
    return 0;
}

#if (CLI_BYPASS == 1)
static int32_t CLI_ByPassApi(CLI_Cfg* ptrCLICfg)
{
    // uint8_t                 cmdString[128];
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus = 0;
    uint32_t                index, idx;
    uint16_t numCLICommands = 0U;

    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;

    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (ptrCLICfg->tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            numCLICommands = numCLICommands + 1;
        }
    }

    /* Execute All Radar Commands */
    for (idx = 0; idx < MAX_RADAR_CMD; idx++)
    {
        /* Reset all the tokenized arguments: */
        memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
        argIndex      = 0;
        ptrCLICommand = (char*)GRadarCmdString[idx];

        if (ptrCLICommand[0] == 0)
        {
            /* End of commands reached */
            return 0;
        }

        /* Set the CLI status: */
        cliStatus = -1;

        /* The command has been entered we now tokenize the command message */
        while (1)
        {
            /* Tokenize the arguments: */
            tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
            if (tokenizedArgs[argIndex] == NULL)
                break;

            /* Increment the argument index: */
            argIndex++;
            if (argIndex >= CLI_MAX_ARGS)
                break;

            /* Reset the command string */
            ptrCLICommand = NULL;
        }

        /* Were we able to tokenize the CLI command? */
        if (argIndex == 0)
            continue;

        /* Cycle through all the registered CLI commands: */
        for (index = 0; index < numCLICommands; index++)
        {
            ptrCLICommandEntry = &ptrCLICfg->tableEntry[index];

            /* Do we have a match? */
            if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
            {
                /* YES: Pass this to the CLI registered function */
                cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
                if (cliStatus == 0)
                {
                    CLI_write ("Done\n\n");
                }
                else
                {
                    CLI_write ("Error %d\n", cliStatus);
                }
                break;
            }
        }

        /* Did we get a matching CLI command? */
        if (index == numCLICommands)
        {
            /* NO matching command found. Is the mmWave extension enabled? */
            if (ptrCLICfg->enableMMWaveExtension == 1U)
            {
                /* Yes: Pass this to the mmWave extension handler */
                cliStatus = CLI_MMWaveExtensionHandler (argIndex, tokenizedArgs);
            }

            /* Was the CLI command found? */
            if (cliStatus == -1)
            {
                /* No: The command was still not found */
                CLI_write ("'%s' is not recognized as a CLI command\r\n", tokenizedArgs[0]);
            }
        }
    }

    return 0;
}
#endif

void CLI_uart_read_callback(UART_Handle handle, UART_Transaction *trans)
{
    DebugP_assertNoLog(UART_TRANSFER_STATUS_SUCCESS == trans->status);
    SemaphoreP_post(&gUartReadDoneSem);

    return;
}

int32_t CLI_readLine(UART_Handle uartHandle, char *lineBuf, uint32_t bufSize)
{
    int32_t status = SystemP_FAILURE;

    if(uartHandle!=NULL)
    {
        uint32_t done = 0;
        UART_Transaction trans;
        uint8_t  readByte;
        int32_t  transferOK;
        uint32_t numCharRead = 0;

        while(!done)
        {
            UART_Transaction_init(&trans);

            status = SystemP_SUCCESS;

            /* Read one char */
            trans.buf   = &readByte;
            trans.count = 1;
            transferOK = UART_read(uartHandle, &trans);

            /* Wait for read completion */
            //SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);

            if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
            {
                status = SystemP_FAILURE;
            }
            if(status == SystemP_SUCCESS)
            {
                if(numCharRead < bufSize)
                {
                    lineBuf[numCharRead] = readByte;
                    numCharRead++;
                }

                /* Echo the char */
                trans.buf   = &readByte;
                trans.count = 1;
                transferOK = UART_write(uartHandle, &trans);
                if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
                {
                    status = SystemP_FAILURE;
                }
            }
            if(status == SystemP_SUCCESS)
            {
                if((readByte == 10) || (readByte == 13))/* "LINE FEED" "New Line" entered, (ASCII: 10, 13) */
                {
                    /* terminate the string, reset numCharRead  */
                    lineBuf[numCharRead-1] = 0;

                    done = 1;

                    /* Echo a new line to terminal (ASCII: 10) */
                    readByte = 10;
                    trans.buf   = &readByte;
                    trans.count = 1;
                    transferOK = UART_write(uartHandle, &trans);
                    if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != trans.status))
                    {
                        status = SystemP_FAILURE;
                    }
                }
            }
            if(status != SystemP_SUCCESS)
            {
                done = 1; /* break out in case of error */
            }
        }
    }
    return status;
}

static int32_t CLI_MMWaveSensorStop (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(gIsSensorStarted == 1)
    {
        gSensorStop = 1;
    }
    /*Setting the GPIO pin to default high state for proper behavior of this pin*/
    //GPIO_pinWriteHigh(gGpioBaseAddrLed, gPinNumLed);
    return 0;
}

static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[])
{
    uint32_t i;
    uint16_t cliRxEnbl;
    uint16_t cliTxEnbl;
    
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    cliRxEnbl = atoi (argv[1]);
    cliTxEnbl = atoi (argv[2]);

    gMmwMssMCB.mmWaveCfg.rxEnbl  = 0;
    gMmwMssMCB.mmWaveCfg.txEnbl  = 0;

    gMmwMssMCB.numRxAntennas = 0;
    gMmwMssMCB.numTxAntennas = 0;
    for (i = 0; i < (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL); i++)
    {
        if((cliTxEnbl >> i) & 0x1)
        {
            if (gMmwMssMCB.numTxAntennas < SYS_COMMON_NUM_TX_ANTENNAS)
            {
                gMmwMssMCB.txAntOrder[gMmwMssMCB.numTxAntennas] = (uint8_t) i;
                gMmwMssMCB.mmWaveCfg.txEnbl |= gTxAntMask[gMmwMssMCB.numTxAntennas];
                gMmwMssMCB.numTxAntennas++;
            }
            else
            {
                CLI_write ("Error: Number of selected Tx antennas must be less than or equal to max Tx\n");
                return -1;
            }
        }
        if((cliRxEnbl >> i) & 0x1)
        {
            if (gMmwMssMCB.numRxAntennas < SYS_COMMON_NUM_RX_CHANNEL)
            {
                gMmwMssMCB.rxAntOrder[gMmwMssMCB.numRxAntennas] = (uint8_t) i;
                gMmwMssMCB.mmWaveCfg.rxEnbl |= gRxAntMask[gMmwMssMCB.numRxAntennas];
                gMmwMssMCB.numRxAntennas++;
            }
            else
            {
                CLI_write ("Error: Number of selected Rx antennas must be less than or equal to max Rx\n");
                return -1;
            }
        }
    }

    /* Save bit masks of enabled tx/rx antennas  */
    gMmwMssMCB.cliRxEnbl = cliRxEnbl;
    gMmwMssMCB.cliTxEnbl = cliTxEnbl;

    return 0;
}

static int32_t CLI_MMWaveChirpCommonCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the Chirp Common configuration: */
    gMmwMssMCB.mmWaveCfg.profileComCfg.digOutputSampRate  = atoi (argv[1]);
    gMmwMssMCB.mmWaveCfg.profileComCfg.digOutputBitsSel   = atoi (argv[2]);
    gMmwMssMCB.mmWaveCfg.profileComCfg.dfeFirSel          = atoi (argv[3]);
    gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples    = atoi (argv[4]);
    gMmwMssMCB.mmWaveCfg.profileComCfg.chirpTxMimoPatSel  = atoi (argv[5]);
    gMmwMssMCB.mmWaveCfg.profileComCfg.chirpRampEndTimeus  = atof (argv[6]); 
    gMmwMssMCB.mmWaveCfg.profileComCfg.chirpRxHpfSel      = atoi (argv[7]);

    gMmwMssMCB.adcSamplingRate = 200.0/gMmwMssMCB.mmWaveCfg.profileComCfg.digOutputSampRate;
    
    if ((gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples >= 2U) && (gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples <= 1024U))
    {
        gMmwMssMCB.rangeFftSize = mathUtils_pow2roundup(gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples);
        gMmwMssMCB.numRangeBins = gMmwMssMCB.rangeFftSize/2; //Real only sampling
    }
    else
    {
        CLI_write ("Error: Number of adc samples configured is not within the supported range\n");
        return -1;
    }

    return 0;
}

static int32_t CLI_MMWaveChirpTimingCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the Chirp Timing configuration: */
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpIdleTimeus    = atof (argv[1]);
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpAdcStartTime  = atoi (argv[2]); //TODO: num of skip samples - to be confirmed based on DFP recommendations
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpTxStartTimeus = atof (argv[3]);
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpSlope         = atof (argv[4]); 
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.startFreqGHz          = atof (argv[5]);

    return 0;
}

static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst      = atoi (argv[1]);
    gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsAccum        = atoi (argv[2]);
    gMmwMssMCB.mmWaveCfg.frameCfg.burstPeriodus           = atof (argv[3]); //us
    gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame      = atoi (argv[4]);
    gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus      = lroundf (1000.0f * atof (argv[5]));
    gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames             = atoi (argv[6]);

    return 0;
}

static int32_t CLI_MMWaveCfarProcCfg (int32_t argc, char* argv[])
{
    uint32_t            procDirection;
    float               threshold;

    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    procDirection                             = (uint32_t) atoi (argv[1]);
    
    if (procDirection == 0)
    {
        gMmwMssMCB.cfarRangeCfg.averageMode       = (uint8_t) atoi (argv[2]);
        gMmwMssMCB.cfarRangeCfg.winLen            = (uint8_t) atoi (argv[3]);
        gMmwMssMCB.cfarRangeCfg.guardLen          = (uint8_t) atoi (argv[4]);
        gMmwMssMCB.cfarRangeCfg.noiseDivShift     = (uint8_t) atoi (argv[5]);
        gMmwMssMCB.cfarRangeCfg.cyclicMode        = (uint8_t) atoi (argv[6]);
        threshold                                 = (float) atof (argv[7]);    
        gMmwMssMCB.cfarRangeCfg.peakGroupingEn    = (uint8_t) atoi (argv[8]);

        if (threshold > 100.0)
        {
            CLI_write("Error: Maximum value for CFAR thresholdScale is 100.0 dB.\n");
            return -1;
        }

        threshold = threshold * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR;

        gMmwMssMCB.cfarRangeCfg.thresholdScale    = (uint16_t) threshold;
    }
    else
    {
        gMmwMssMCB.cfarDopplerCfg.averageMode       = (uint8_t) atoi (argv[2]);
        gMmwMssMCB.cfarDopplerCfg.winLen            = (uint8_t) atoi (argv[3]);
        gMmwMssMCB.cfarDopplerCfg.guardLen          = (uint8_t) atoi (argv[4]);
        gMmwMssMCB.cfarDopplerCfg.noiseDivShift     = (uint8_t) atoi (argv[5]);
        gMmwMssMCB.cfarDopplerCfg.cyclicMode        = (uint8_t) atoi (argv[6]);
        threshold                                   = (float) atof (argv[7]);    
        gMmwMssMCB.cfarDopplerCfg.peakGroupingEn    = (uint8_t) atoi (argv[8]);

        if (threshold > 100.0)
        {
            CLI_write("Error: Maximum value for CFAR thresholdScale is 100.0 dB.\n");
            return -1;
        }

        threshold = threshold * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR;

        gMmwMssMCB.cfarDopplerCfg.thresholdScale    = (uint16_t) threshold;
    }

    return 0;
}

static int32_t CLI_MMWaveCfarFovCfg (int32_t argc, char* argv[])
{
    uint32_t            procDirection;

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    procDirection                              = (uint32_t) atoi (argv[1]);
    
    if (procDirection == 0)
    {
        gMmwMssMCB.fovRange.min                = (float) atof (argv[2]);
        gMmwMssMCB.fovRange.max                = (float) atof (argv[3]);
    }
    else
    {
        gMmwMssMCB.fovDoppler.min              = (float) atof (argv[2]);
        gMmwMssMCB.fovDoppler.max              = (float) atof (argv[3]);
    }

    return 0;
}

static int32_t CLI_MMWaveAoaProcCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.aoaProcCfg.azimuthFftSize   = (uint16_t) atoi (argv[1]);
    gMmwMssMCB.aoaProcCfg.elevationFftSize = (uint16_t) atoi (argv[2]);
    
    return 0;
}

static int32_t CLI_MMWaveAoaCfg (int32_t argc, char* argv[])
{

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Major Motion FOV configuration: */
    gMmwMssMCB.fovAoaCfg.minAzimuthDeg      = (float) atof (argv[1]);
    gMmwMssMCB.fovAoaCfg.maxAzimuthDeg      = (float) atof (argv[2]);
    gMmwMssMCB.fovAoaCfg.minElevationDeg    = (float) atof (argv[3]);
    gMmwMssMCB.fovAoaCfg.maxElevationDeg    = (float) atof (argv[4]);

    /* Minor Motion FOV configuration*/
    gMmwMssMCB.dspPreStartCfgLocal.fovCfg[0] = (float)atof(argv[2]);
    gMmwMssMCB.dspPreStartCfgLocal.fovCfg[1] = (float)atof(argv[4]);
    return 0;
}

#if 0
static int32_t CLI_MMWaveGuiMonSel (int32_t argc, char* argv[])
{

    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.guiMonSel.pointCloud           = atoi (argv[1]);
    gMmwMssMCB.guiMonSel.rangeProfile         = atoi (argv[2]);
    gMmwMssMCB.guiMonSel.noiseProfile         = atoi (argv[3]);
    gMmwMssMCB.guiMonSel.rangeAzimuthHeatMap  = atoi (argv[4]);
    gMmwMssMCB.guiMonSel.rangeDopplerHeatMap  = atoi (argv[5]);
    gMmwMssMCB.guiMonSel.statsInfo            = atoi (argv[6]);
    //gMmwMssMCB.guiMonSel.adcSamples           = atoi (argv[7]);

    return 0;
}

static int32_t CLI_MMWaveMultiObjBeamForming (int32_t argc, char* argv[])
{

    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.multiObjBeamFormingCfg.enabled                = (uint8_t) atoi (argv[1]);
    gMmwMssMCB.multiObjBeamFormingCfg.multiPeakThrsScal      = (float) atof (argv[2]);
    
    return 0;
}

static int32_t CLI_MMWaveRangeSelCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.rangeSelCfg.min               = (float) atof (argv[1]);
    gMmwMssMCB.rangeSelCfg.max               = (float) atof (argv[2]);

    return 0;
}
#endif

static int32_t CLI_MMWaveClutterRemoval (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.staticClutterRemovalEnable          = (bool) atoi (argv[1]);
    
    return 0;
}

static int32_t CLI_MMWaveLowPwrModeEnable(int32_t argc, char* argv[])
{
    if (argc != 2)
    {
             CLI_write ("Error: Invalid usage of the CLI command\n");
             return -1;
    }

    gMmwMssMCB.lowPowerMode = atoi (argv[1]);

    return 0;
 }

static int32_t CLI_MMWaveFactoryCalConfig (int32_t argc, char* argv[])
{
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\r\n");
        return -1;
    }
    /* Populate configuration: */
    gMmwMssMCB.mmWaveCfg.calibCfg.saveEnable    = (uint32_t) atoi(argv[1]);
    gMmwMssMCB.mmWaveCfg.calibCfg.restoreEnable = (uint32_t) atoi(argv[2]);
    gMmwMssMCB.mmWaveCfg.calibCfg.rxGain        = (uint32_t) atoi(argv[3]);
    gMmwMssMCB.mmWaveCfg.calibCfg.txBackoffSel  = (uint32_t) atoi(argv[4]);
    sscanf(argv[5], "0x%x", &gMmwMssMCB.mmWaveCfg.calibCfg.flashOffset);
    /* Validate inputs */
    /* <Save> and <re-store> shouldn't be enabled in CLI*/
    if ((gMmwMssMCB.mmWaveCfg.calibCfg.saveEnable == 1) && (gMmwMssMCB.mmWaveCfg.calibCfg.restoreEnable == 1))
    {
        CLI_write ("Error: Save and Restore can be enabled only one at a time\r\n");
        return -1;
    }
    /* Validate inputs */
    /* RxGain should be between 38db to 46db */
    if ( (gMmwMssMCB.mmWaveCfg.calibCfg.rxGain > 46U) || (gMmwMssMCB.mmWaveCfg.calibCfg.rxGain < 38U))
    {
        CLI_write ("Error: Valid RxGain should be between 38db to 46db\r\n");
        return -1;
    }
    /* txBackoffSel should be between 0db to 26db */
    if ((uint32_t) (gMmwMssMCB.mmWaveCfg.calibCfg.txBackoffSel) > 26U)
    {
        CLI_write ("Error: Valid txBackoffSel should be between 0db to 26db\r\n");
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for antenna geometry configuration
 *      Arguments are row/column coordinates of the virtual antennas in 
 *      units of lambda/2. The arguments are 
 *      <virtAnt0_row> <virtAnt0_col> <virtAnt1_row> <virtAnt1_col> ... <virtAnt15_row> <virtAnt15_col> <antennaDistanceXdim> <antennaDistanceZdim>
 *      where 
 *           virtAnt0 corresponds to tx0-rx0,
 *           virtAnt1 corresponds to tx0-rx1,
 *           virtAnt2 corresponds to tx0-rx2,
 *           virtAnt3 corresponds to tx0-rx3,
 *           virtAnt4 corresponds to tx1-rx1,
 *           ....
 *           ....
 *           virtAnt15 corresponds to tx3-rx3
 *      The last two parameters are optional and represent antenna spacing (mm)
 *         <antennaDistanceXdim> antenna spacing in X dimension
 *         <antennaDistanceZdim> antenna spacing in Z dimension
 *      If these two arguments are not specified, it is assumed that lambda/d=2 where d is distance beetween antennas.
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MmwDemo_AntGeometryCfg (int32_t argc, char* argv[])
{
        MmwDemo_antennaGeometryCfg   cfg;
        int32_t argInd;
        int32_t i;

        /* Sanity Check: Minimum argument check */
        if ((argc < (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2)) ||
            (argc > (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 2)))
        {
            CLI_write ("Error: Invalid usage of the CLI command\n");
            return -1;
        }

        /* Initialize configuration: */
        memset ((void *)&cfg, 0, sizeof(cfg));


        argInd = 1;
        for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
        {
            cfg.ant[i].row  = (int8_t) atoi(argv[argInd++]);
            cfg.ant[i].col  = (int8_t) atoi(argv[argInd++]);

            gMmwMssMCB.dspPreStartCfgLocal.m_ind[i] = -cfg.ant[i].col;
            gMmwMssMCB.dspPreStartCfgLocal.n_ind[i] = -cfg.ant[i].row;
        }

        /* Check if antenna spacings in X-dimesnsion is present  */
        if (argc > (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
        {
            cfg.antDistanceXdimMts = atof(argv[1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 0]) * 1e-3; //Saved in meters
        }
        /* Check if antenna spacings in Z-dimension is present  */
        if (argc > (1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 1))
        {
            cfg.antDistanceZdimMts = atof(argv[1 + SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2 + 1]) * 1e-3; //Saved in meters
        }

        /* Save Configuration to use later */
        memcpy((void *) &gMmwMssMCB.antennaGeometryCfg, &cfg, sizeof(cfg));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for antenna geometry and gain bias and phase compensation configuration
 *      Argument is the board name
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MmwDemo_AntGeometryBoard (int32_t argc, char* argv[])
{
    DPU_AoAProc_compRxChannelBiasCfg   rangePhaseCfg;
    /* Arrays to store the TX and RX antenna indices */
    int8_t antGeometryTX[2*SYS_COMMON_NUM_TX_ANTENNAS];
    int8_t antGeometryRX[2*SYS_COMMON_NUM_RX_CHANNEL];
    int32_t txInd,
            rxInd,
            antInd,
            Re,
            Im;
    char *token;

    /* Initialize configuration: */
    memset ((void *)&rangePhaseCfg, 0, sizeof(rangePhaseCfg));

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    if((GIsAntGeoDef >> 3) == 1)
    {
        CLI_write ("Error: Antenna geometry is already defined\n");
        return -1;
    }

    if(GIsRangePhaseCompDef == 1)
    {
        CLI_write ("Error: Range bias and phase compensation is already defined\n");
        return -1;
    }

    if(strcmp(argv[1], "xWRL6844EVM") == 0)
    {
        /* A value of 1000b in GIsAntGeoDef indicates its fully defined */
        GIsAntGeoDef = GIsAntGeoDef << 3;
        /* range bias and phase compensation is fully defined */
        GIsRangePhaseCompDef  = 1;

        /* Function to extract and process antGeometryTX string */
        /* Skip first word */
        token = strtok(GantGeoRangePhaseCompxWRL6844EVM[0], " \r\n");

        for(txInd = 0; txInd < 2*SYS_COMMON_NUM_TX_ANTENNAS; txInd++)
        {
            token = strtok(NULL, " \r\n");
            antGeometryTX[txInd] = (int8_t) atoi(token);
        }
        
        /* Function to extract and process antGeometryRX string */
        /* Skip first word */
        token = strtok(GantGeoRangePhaseCompxWRL6844EVM[1], " \r\n");

        for(rxInd = 0; rxInd < 2*SYS_COMMON_NUM_RX_CHANNEL; rxInd++)
        {
            token = strtok(NULL, " \r\n");
            antGeometryRX[rxInd] = (int8_t) atoi(token);
        }

        /* Function to extract and process antGeometryDist string */
        /* Skip first word */
        token = strtok(GantGeoRangePhaseCompxWRL6844EVM[2], " \r\n");
        /* Move to first number */
        token = strtok(NULL, " \r\n");

        /* Antenna spacings in X-dimesnsion in meters */
        gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts = atof(token) * 1e-3;
        token = strtok(NULL, " \r\n");
        /* Antenna spacings in Z-dimesnsion in meters */
        gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts = atof(token) * 1e-3;

        antInd = 0;
        for(txInd = 0; txInd < 2*SYS_COMMON_NUM_TX_ANTENNAS; txInd+=2)
        {
            for(rxInd = 0; rxInd < 2*SYS_COMMON_NUM_RX_CHANNEL; rxInd+=2)
            {
                gMmwMssMCB.antennaGeometryCfg.ant[antInd].row = antGeometryTX[txInd] + antGeometryRX[rxInd];
                gMmwMssMCB.antennaGeometryCfg.ant[antInd].col = antGeometryTX[txInd + 1] + antGeometryRX[rxInd + 1];
                antInd++;
            }
        }

        /* Function to extract and process compRangeBiasAndRxChanPhase string */
        /* Skip first word */
        token = strtok(GantGeoRangePhaseCompxWRL6844EVM[3], " \r\n");
        /* Move to first number */
        token = strtok(NULL, " \r\n");

        rangePhaseCfg.rangeBias          = (float) atof (token);

        for (antInd=0; antInd < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; antInd++)
        {
            token = strtok(NULL, " \r\n");
            Re = (int32_t) (atof (token) * 32768.);
            MATHUTILS_SATURATE16(Re);
            rangePhaseCfg.rxChPhaseComp[antInd].real = (int16_t) Re;

            token = strtok(NULL, " \r\n");
            Im = (int32_t) (atof (token) * 32768.);
            MATHUTILS_SATURATE16(Im);
            rangePhaseCfg.rxChPhaseComp[antInd].imag = (int16_t) Im;
        }

        gMmwMssMCB.compRxChannelBiasCfgMajor.rangeBias = rangePhaseCfg.rangeBias;

        for(txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
        {
            for(rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
            {
                gMmwMssMCB.compRxChannelBiasCfgMajor.rxChPhaseComp[txInd * gMmwMssMCB.numRxAntennas + rxInd] =
                    rangePhaseCfg.rxChPhaseComp[gMmwMssMCB.txAntOrder[txInd] * SYS_COMMON_NUM_RX_CHANNEL +
                                                                    gMmwMssMCB.rxAntOrder[rxInd]];
            }
        }
    }
    else
    {
        CLI_write ("Error: Board is not supported\n");
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.measureRxChannelBiasCliCfg.enabled          = (uint8_t) atoi (argv[1]);
    gMmwMssMCB.measureRxChannelBiasCliCfg.targetDistanceMts   = (float) atof (argv[2]);
    gMmwMssMCB.measureRxChannelBiasCliCfg.searchWinSizeMts    = (float) atof (argv[3]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Converts float to 21-bit signed integer
 *
 *  @param[in] x
 *      floating point vlue
 *
 *  @retval
 *      y converted input value to integer , saturated to 21bit wide signed integer
 */

static int32_t f2i21(float x)
{
    int32_t y;
    y = x < 0 ? (int32_t) (x - 0.5) : (int32_t) (x + 0.5);
    y = y > 1048575 ? 1048575 : y;
    y = y < -1048575 ? -1048575 : y;
    return (int32_t) y;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
//volatile uint32_t gRxCompCfgCntr = 0;

#define LOC_VAR_OUSIDE 1
#if LOC_VAR_OUSIDE == 1
static DPU_AoAProc_compRxChannelBiasCfg cfg;
static int32_t argInd;
static int32_t i;
static float absMax, absMaxInverse;
static float absVal;
#endif

// Major
// static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
// {
//     DPU_AoAProc_compRxChannelBiasCfg   cfg;
//     int32_t Re, Im;
//     int32_t argInd;
//     int32_t txInd,rxInd,antInd;

//     /* Sanity Check: Minimum argument check */
//     if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     if(GIsRangePhaseCompDef == 1)
//     {
//         CLI_write ("Error: Range bias and phase compensation is already defined\n");
//         return -1;
//     }

//     /* range bias and phase compensation is fully defined */
//     GIsRangePhaseCompDef  = 1;

//     /* Initialize configuration: */
//     memset ((void *)&cfg, 0, sizeof(cfg));

//     /* Populate configuration: */
//     cfg.rangeBias          = (float) atof (argv[1]);

//     argInd = 2;
//     for (antInd=0; antInd < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; antInd++)
//     {
//         Re = (int32_t) (atof (argv[argInd++]) * 32768.);
//         MATHUTILS_SATURATE16(Re);
//         cfg.rxChPhaseComp[antInd].real = (int16_t) Re;

//         Im = (int32_t) (atof (argv[argInd++]) * 32768.);
//         MATHUTILS_SATURATE16(Im);
//         cfg.rxChPhaseComp[antInd].imag = (int16_t) Im;
//     }
    
//     gMmwMssMCB.compRxChannelBiasCfg.rangeBias = cfg.rangeBias;

//     for(txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
//     {
//         for(rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
//         {
//             gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[txInd * gMmwMssMCB.numRxAntennas + rxInd] =
//                 cfg.rxChPhaseComp[gMmwMssMCB.txAntOrder[txInd] * SYS_COMMON_NUM_RX_CHANNEL +
//                                                                 gMmwMssMCB.rxAntOrder[rxInd]];
//         }
//     }

//     return 0;
// }

// Incabin
static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
#if LOC_VAR_OUSIDE == 0
    DPU_Doa3dProc_compRxChannelBiasCfg   cfg;
    int32_t argInd;
    int32_t i;
    float absMax, absMaxInverse;
    float absVal;
#endif
    float *phaseCompVect = gMmwMssMCB.dspPreStartCfgLocal.phaseCompVect; //For Minor Motion cfg

/*
    gRxCompCfgCntr++;
    while (gRxCompCfgCntr == 2)
    {
        ;
    }
*/
    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < 2*SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {

        absVal = fabs(atof(argv[argInd++]));
        if (absVal > absMax)
        {
            absMax = absVal;
        }
    }
    if (absMax > 0.)
    {
        absMaxInverse = 1./absMax;
    }
    else
    {
        absMaxInverse = 1.;
    }
    absMaxInverse = absMaxInverse * 1048576.; //Q20

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        float cRe, cIm;
        cRe = (atof (argv[argInd++]) * absMaxInverse);
        cIm = (atof (argv[argInd++]) * absMaxInverse);
        cfg.rxChPhaseComp[i].real = f2i21(cRe);
        cfg.rxChPhaseComp[i].imag = f2i21(cIm);
        phaseCompVect[2*i]   = cIm * 9.53674316e-07; // 9.53674316e-07 = 1/2^20
        phaseCompVect[2*i+1] = cRe * 9.53674316e-07;
    }

    gMmwMssMCB.compRxChannelBiasCfg = cfg;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for ADC data source configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MMWaveAdcDataSourceCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.adcDataSourceCfg.source = atoi (argv[1]);
    
    if (strlen(argv[2]) <= DPC_ADC_FILENAME_MAX_LEN)
    {
        strncpy(gMmwMssMCB.adcDataSourceCfg.fileName, argv[2], DPC_ADC_FILENAME_MAX_LEN);
    }
    else
    {
        CLI_write ("Error: Filename too long\n");
        return -1;
    }

    return 0;
}

static int32_t CLI_MMWaveAdcLogging (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if ((argc != 2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.adcLogging.enable = (uint8_t) atoi (argv[1]);
   
    return 0;
}

int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the SensorStop configuration: */
    gMmwMssMCB.mmWaveCfg.strtCfg.frameTrigMode      = atoi (argv[1]);
    gMmwMssMCB.mmWaveCfg.strtCfg.chirpStartSigLbEn  = atoi (argv[2]);
    gMmwMssMCB.mmWaveCfg.strtCfg.frameLivMonEn      = atoi (argv[3]);
    gMmwMssMCB.mmWaveCfg.strtCfg.frameTrigTimerVal  = atoi (argv[4]);

    MmwDemo_startDemoProcessing();

    return 0;
}

static int32_t CLI_MMWaveRuntimeCalConfig (int32_t argc, char* argv[])
{
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\r\n");
        return -1;
    }
    /* Populate configuration: */
    gMmwMssMCB.mmWaveCfg.openCfg.runTxCLPCCalib = (bool) atoi (argv[1]);

    return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
//     Add below new commands for
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *  @b Description
 *  @n
 *      This is the CLI for Running mode selection
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// static int32_t MmwDemo_CLIRunningModeCfg(int32_t argc, char *argv[])
// {

//     /* Sanity Check: */
//     if ((argc != (1 + 1)) && (argc != (1 + 2)))
//     {
//         CLI_write("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     gMmwMssMCB.runningMode = (uint32_t) atoi(argv[1]);

//     if (argc >= (1 + 2))
//     {
//         gMmwMssMCB.cpdOption = (uint32_t) atoi(argv[2]);
//     }

//     return 0;
// }

// /**
//  *  @b Description
//  *  @n
//  *      This is the CLI Handler for MacroDoppler Steering Debug Configuration
//  *
//  *  @param[in] argc
//  *      Number of arguments
//  *  @param[in] argv
//  *      Arguments
//  *
//  *  @retval
//  *      Success -   0
//  *  @retval
//  *      Error   -   <0
//  */
// int32_t MmwDemo_CLIMacroDopplerSteerDbgCfg (int32_t argc, char* argv[])
// {

//     MmwDemo_MacroDopplerSteerDbgCfg   cfg;

//     if(argc != (1+1))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Initialize configuration: */
//     memset ((void *)&cfg, 0, sizeof(cfg));

//     /* Populate configuration: */
//     cfg.enabled  = (uint16_t) atoi (argv[1]);

//     /* Save Configuration to use later */
//     gMmwMssMCB.cliMacroDopplerSteerDbgCfg = cfg;

//     return 0;
// }

// /**
//  *  @b Description
//  *  @n
//  *      This is the CLI Handler for Debugging Point cloud generation
//  *
//  *  @param[in] argc
//  *      Number of arguments
//  *  @param[in] argv
//  *      Arguments
//  *
//  *  @retval
//  *      Success -   0
//  *  @retval
//  *      Error   -   <0
//  */
// int32_t MmwDemo_CLIPointCloudGenerationDbgCfg (int32_t argc, char* argv[])
// {

//     MmwDemo_PointCloudGenerationDbgCfg   cfg;

//     if(argc != (1+1))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Initialize configuration: */
//     memset ((void *)&cfg, 0, sizeof(cfg));

//     /* Populate configuration: */
//     cfg.disablePointCloudGeneration  = (uint16_t) atoi (argv[1]);

//     /* Save Configuration to use later */
//     gMmwMssMCB.cliPointCloudGenDbgCfg = cfg;

//     return 0;
// }

// /**
//  *  @b Description
//  *  @n
//  *      This is the CLI Handler for MacroDoppler Configuration
//  *
//  *  @param[in] argc
//  *      Number of arguments
//  *  @param[in] argv
//  *      Arguments
//  *
//  *  @retval
//  *      Success -   0
//  *  @retval
//  *      Error   -   <0
//  */
// int32_t MmwDemo_CLIMacroDopplerCfg (int32_t argc, char* argv[])
// {

//     MmwDemo_MacroDopplerCfg   cfg;

//     if ((argc < (1+2)) || (argc > (1+8)))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Initialize configuration: */
//     memset ((void *)&cfg, 0, sizeof(cfg));
//     /* Populate configuration: */
//     cfg.macroDopplerFeatureEnabled  = (uint16_t) atoi (argv[1]);
//     cfg.multiFrmDelayLineLen        = (uint16_t) atoi (argv[2]);
//     cfg.multiFrmPhaseDopFftEnabled  =   0; 
//     cfg.multiFrmFftWindowEnabled    =   0; 
//     cfg.multiFrmDopplerFftSize      = 128; 
//     cfg.numAvgChirpsPerFrame        =  32; 
//     cfg.chirpAvgStartIdx            =   0; 
//     cfg.chirpAvgStep                =   1; 

//     if (argc >= 4)
//     {
//         cfg.multiFrmPhaseDopFftEnabled  =  (uint16_t) atoi (argv[3]);
// 	}
//     if (argc >= 5)
//     {
//         cfg.multiFrmFftWindowEnabled    =  (uint16_t) atoi (argv[4]);
// 	}
//     if (argc >= 6)
//     {
//         cfg.multiFrmDopplerFftSize      =  (uint16_t) atoi (argv[5]);
// 	}
//     if (argc >= 7)
//     {
// 	    cfg.numAvgChirpsPerFrame        =  (uint16_t) atoi (argv[6]);
// 	}
//     if (argc >= 8)
//     {
//         cfg.chirpAvgStartIdx            =  (uint16_t) atoi (argv[7]);
// 	}
//     if (argc >= 9)
//     {
// 	    cfg.chirpAvgStep                =  (uint16_t) atoi (argv[8]);		
// 	}
//     /* Save Configuration to use later */
//     gMmwMssMCB.cliMacroDopplerCfg = cfg;

//     return 0;
// }

/**
 *  @b Description
 *  @n
 *      This is the CLI for Macro Doppler Map (input to CNN) scale configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// static int32_t MmwDemo_CLIMacroDopplerMapScaleCfg(int32_t argc, char *argv[])
// {
//     /* Sanity Check: */
//     if (argc > (1 + FEXTRACT_MAX_OCCUPANCY_BOXES))
//     {
//         CLI_write("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     uint8_t boxIdx;
//     for (boxIdx = 0; boxIdx < argc-1; boxIdx++)
//     {
//         gMmwMssMCB.cliMacroDopplerMapScale.cnnInputScale[boxIdx] = (float) atof (argv[boxIdx+1]);
//     }
//     gMmwMssMCB.cliMacroDopplerMapScaleCmdPending = 1;
//     return 0;
// }

// /**
//  *  @b Description
//  *  @n
//  *      This is the CLI for Macro Doppler - number of voxel points per zone
//  *
//  *  @param[in] argc
//  *      Number of arguments
//  *  @param[in] argv
//  *      Arguments
//  *
//  *  @retval
//  *      Success -   0
//  *  @retval
//  *      Error   -   <0
//  */
// static int32_t MmwDemo_CLIMacroDopplerNumVoxelCfg(int32_t argc, char *argv[])
// {
//     /* Sanity Check: */
//     if (argc > (1 + FEXTRACT_MAX_OCCUPANCY_BOXES))
//     {
//         CLI_write("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     uint8_t boxIdx;
//     for (boxIdx = 0; boxIdx < argc-1; boxIdx++)
//     {
//         gMmwMssMCB.cliMacroDopplerNumVoxel.numVoxel[boxIdx] = (uint8_t) atoi (argv[boxIdx+1]);

//         if (gMmwMssMCB.cliMacroDopplerNumVoxel.numVoxel[boxIdx] & 0x1)
//         {
//             CLI_write("Error: Number of Voxels per zone had to be even\n");
//             return -1;
//         }
//     }
//     return 0;
// }

// /**
//  *  @b Description
//  *  @n
//  *      This is the CLI for Macro Doppler - range bin offsets relative to minimum distance of the
//  *
//  *  @param[in] argc
//  *      Number of arguments
//  *  @param[in] argv
//  *      Arguments
//  *
//  *  @retval
//  *      Success -   0
//  *  @retval
//  *      Error   -   <0
//  */
// static int32_t MmwDemo_CLIMacroDopplerRngBinOffsCfg(int32_t argc, char *argv[])
// {
//     /* Sanity Check: */
//     if (argc > (1 + FEXTRACT_MAX_OCCUPANCY_BOXES))
//     {
//         CLI_write("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     uint8_t boxIdx;
//     for (boxIdx = 0; boxIdx < argc-1; boxIdx++)
//     {
//         gMmwMssMCB.cliMacroDopplerRngBinOffs.rngBinOffs[boxIdx] = (int8_t) atoi (argv[boxIdx+1]);
//     }
//     return 0;
// }


// int32_t MmwDemo_CLIOccupancyBoxCfg (int32_t argc, char* argv[])
// {
//     int32_t boxInd;
//     /* Sanity Check: Minimum argument check */
//     if (argc != 8)
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Populate configuration: */
//     boxInd = atoi (argv[1]);
//     if (!(boxInd < IDETECT_MAX_OCCUPANCY_BOXES))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     gMmwMssMCB.idetSceneryParams.occupancyBox[boxInd].x1 = (float) atof (argv[2]);
//     gMmwMssMCB.idetSceneryParams.occupancyBox[boxInd].x2 = (float) atof (argv[3]);
//     gMmwMssMCB.idetSceneryParams.occupancyBox[boxInd].y1 = (float) atof (argv[4]);
//     gMmwMssMCB.idetSceneryParams.occupancyBox[boxInd].y2 = (float) atof (argv[5]);
//     gMmwMssMCB.idetSceneryParams.occupancyBox[boxInd].z1 = (float) atof (argv[6]);
//     gMmwMssMCB.idetSceneryParams.occupancyBox[boxInd].z2 = (float) atof (argv[7]);
//     gMmwMssMCB.idetSceneryParams.numOccupancyBoxes = boxInd + 1;
    
//     return 0;
// }

// int32_t MmwDemo_CLIIntruderDetCfg (int32_t argc, char* argv[])
// {
//     int32_t boxInd;
//     /* Sanity Check: Minimum argument check */
//     if (argc != 7)
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Copy the same parameter to each occupancy box */
//     for (boxInd = 0; boxInd < gMmwMssMCB.idetSceneryParams.numOccupancyBoxes; boxInd++)
//     {
//         gMmwMssMCB.idetStateParams.occupancyThre[boxInd] = (float) atof (argv[1]);
//         gMmwMssMCB.idetStateParams.free2activeThre[boxInd] = (uint16_t) atoi (argv[2]);
//         gMmwMssMCB.idetStateParams.active2freeThre[boxInd] = (uint16_t) atoi (argv[3]);

//         gMmwMssMCB.idetSigProcParams.localPeakCheck[boxInd] = (uint8_t) atoi (argv[4]);
//         gMmwMssMCB.idetSigProcParams.sidelobeThre[boxInd] = (float) atof (argv[5]);
//         gMmwMssMCB.idetSigProcParams.peakExpSamples[boxInd] = (uint8_t) atoi (argv[6]);
//     }
    
//     return 0;
// }

// int32_t MmwDemo_CLIIntruderDetAdvCfg (int32_t argc, char* argv[])
// {
//     int32_t boxInd;
//     /* Sanity Check: Minimum argument check */
//     if (argc != 8)
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Populate configuration: */
//     boxInd = atoi (argv[1]);
//     if (!(boxInd < gMmwMssMCB.idetSceneryParams.numOccupancyBoxes))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     gMmwMssMCB.idetStateParams.occupancyThre[boxInd] = (float) atof (argv[2]);
//     gMmwMssMCB.idetStateParams.free2activeThre[boxInd] = (uint16_t) atoi (argv[3]);
//     gMmwMssMCB.idetStateParams.active2freeThre[boxInd] = (uint16_t) atoi (argv[4]);

//     gMmwMssMCB.idetSigProcParams.localPeakCheck[boxInd] = (uint8_t) atoi (argv[5]);
//     gMmwMssMCB.idetSigProcParams.sidelobeThre[boxInd] = (float) atof (argv[6]);
//     gMmwMssMCB.idetSigProcParams.peakExpSamples[boxInd] = (uint8_t) atoi (argv[7]);
    
//     return 0;
// }


// int32_t MmwDemo_CLISensorPositionCfg(int32_t argc, char* argv[])
// {
//     if (argc != 6)
//     {
//         CLI_write("Error: Invalid usage of the CLI Command\n");
//         return -1;
//     }


//     gMmwMssMCB.idetSceneryParams.sensorPosition.x = (float) atof (argv[1]);
//     gMmwMssMCB.idetSceneryParams.sensorPosition.y = (float) atof (argv[2]);
//     gMmwMssMCB.idetSceneryParams.sensorPosition.z = (float) atof (argv[3]);

//     gMmwMssMCB.idetSceneryParams.sensorOrientation.zTilt = ((float) atof (argv[4])) * DEG2RAD; //azimuth
//     gMmwMssMCB.idetSceneryParams.sensorOrientation.yTilt = 0.;
//     gMmwMssMCB.idetSceneryParams.sensorOrientation.xTilt = ((float) atof (argv[5])) * DEG2RAD; //elevation

//     return 0;
// }


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    
    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.guiMonSel.pointCloud           = atoi (argv[1]);
    gMmwMssMCB.guiMonSel.rangeProfile         = atoi (argv[2]);
    gMmwMssMCB.guiMonSel.noiseProfile         = atoi (argv[3]);
    gMmwMssMCB.guiMonSel.rangeAzimuthHeatMap  = atoi (argv[4]);
    gMmwMssMCB.guiMonSel.rangeDopplerHeatMap  = atoi (argv[5]);
    gMmwMssMCB.guiMonSel.statsInfo            = atoi (argv[6]);
    gMmwMssMCB.guiMonSel.trackerInfo          = atoi (argv[7]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration for testing/debugging
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// int32_t MmwDemo_CLIDebugGuiMonSel (int32_t argc, char* argv[])
// {
//     MmwDemo_DbgGuiMonSel   dbgGuiMonSel;

//     //ToDo: this is temporary, put back once the number of args is finalized
//     if (argc != (9+1))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Initialize the guiMonSel configuration: */
//     memset ((void *)&dbgGuiMonSel, 0, sizeof(MmwDemo_DbgGuiMonSel));

//     /* Populate configuration: */
//     dbgGuiMonSel.dbgDetMat3D              = atoi (argv[1]);
//     dbgGuiMonSel.dbgSnr3D                 = atoi (argv[2]);
//     dbgGuiMonSel.dbgAntGeometry           = atoi (argv[3]);
//     dbgGuiMonSel.dbgDetMatSlice           = atoi (argv[4]);
//     dbgGuiMonSel.dbgSnrMatSlice           = atoi (argv[5]);

//     dbgGuiMonSel.exportCoarseHeatmap      =  atoi (argv[6]);
//     dbgGuiMonSel.exportRawCfarDetList     =  atoi (argv[7]);
//     dbgGuiMonSel.exportZoomInHeatmap      =  atoi (argv[8]);

//     dbgGuiMonSel.radCubeFreshChunk        =  atoi (argv[9]);

//     gMmwMssMCB.dbgGuiMonSel = dbgGuiMonSel;
//     return 0;
// }

// /**
//  *  @b Description
//  *  @n
//  *      Configure which "virtual" chirps from radar cube to be exported to host via UART
//  *
//  */
// int32_t MmwDemo_CLIExportRadCubeChunkCfg (int32_t argc, char* argv[])
// {
//     MmwDemo_ExportRadarCubeChunkCfg exportRadarCubeChunkCfg;

//     //ToDo: this is temporary, put back once the number of args is finalized
//     if (argc != (3+1))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         //return -1;
//     }

//     /* Initialize the guiMonSel configuration: */
//     memset ((void *)&exportRadarCubeChunkCfg, 0, sizeof(MmwDemo_ExportRadarCubeChunkCfg));

//     /* Populate configuration: */
//     exportRadarCubeChunkCfg.chirpStartIdx            = atoi (argv[1]);
//     exportRadarCubeChunkCfg.chirpStepIdx             = atoi (argv[2]);
//     exportRadarCubeChunkCfg.numChirps                = atoi (argv[3]);

//     gMmwMssMCB.exportRadarCubeChunkCfg = exportRadarCubeChunkCfg;
//     return 0;
// }

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
    DPU_SNR3DProc_CfarCfg   cfarCfg;

    if(argc != (5 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(cfarCfg));

    /* Populate configuration: */
    cfarCfg.averageMode       = (uint8_t) atoi (argv[1]);
    cfarCfg.winLen            = (uint8_t) atoi (argv[2]);
    cfarCfg.guardLen          = (uint8_t) atoi (argv[3]);
    cfarCfg.noiseDivShift     = (uint8_t) atoi (argv[4]);
    cfarCfg.cyclicMode        = (uint8_t) atoi (argv[5]);

    //ToDo: these fields not used, remove his fields:
    cfarCfg.peakGroupingEn    = 0;
    cfarCfg.sideLobeThresholdScaleQ8 = 0;
    cfarCfg.enableLocalMaxRange      = 0;
    cfarCfg.enableLocalMaxAzimuth    = 0;
    cfarCfg.enableInterpRangeDom     = 0;
    cfarCfg.enableInterpAzimuthDom   = 0;



    cfarCfg.thresholdScale    = (uint32_t) 0;

    gMmwMssMCB.snr3dCfarCfg = cfarCfg;

    return 0;
}

int32_t MmwDemo_CLICfarScndPassCfg (int32_t argc, char* argv[])
{
    DPU_SNR3DProc_CfarScndPassCfg   cfarScndPassCfg;
    /* Sanity Check: Minimum argument check */
    if (argc != (6 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    cfarScndPassCfg.enabled           = (uint8_t) atoi (argv[1]);
    cfarScndPassCfg.averageMode       = (uint8_t) atoi (argv[2]);
    cfarScndPassCfg.winLen            = (uint8_t) atoi (argv[3]);
    cfarScndPassCfg.guardLen          = (uint8_t) atoi (argv[4]);
    cfarScndPassCfg.noiseDivShift     = (uint8_t) atoi (argv[5]);
    cfarScndPassCfg.cyclicMode        = (uint8_t) atoi (argv[6]);

    //ToDo: remove these fields, not used
    cfarScndPassCfg.threshold_dB      = 0;
    cfarScndPassCfg.peakGroupingEn    = 0;

    gMmwMssMCB.snr3dCfarScndPassCfg = cfarScndPassCfg;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for signal processing chain configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// int32_t MmwDemo_CLIIntrusionSigProcChainCfg (int32_t argc, char* argv[])
// {

//     MmwDemo_IntrusionSigProcChainCfg   cfg;

//     if(argc != (3+1))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     /* Initialize configuration: */
//     memset ((void *)&cfg, 0, sizeof(cfg));

//     /* Populate configuration: */
//     cfg.azimuthFftSize                  = (uint16_t) atoi (argv[1]);
//     cfg.elevationFftSize                = (uint16_t) atoi (argv[2]);
//     cfg.selectCoherentPeakInDopplerDim  = (uint8_t)  atoi (argv[3]);

//     /* Save Configuration to use later */
//     gMmwMssMCB.intrusionSigProcChainCfg = cfg;

//     return 0;
// }

/**************************************************************************************************/
/* CLI commands for Minor Motion - Capon                                                               */
/**************************************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for dynamic scene RA CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLINumFrmPerSlidingWindowMinor(int32_t argc, char *argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    gMmwMssMCB.numFramesPerMinorMode       = (uint8_t)atoi(argv[1]);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for dynamic scene RA CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIDynRACfarCfg(int32_t argc, char *argv[])
{
    if (argc != (15 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.leftSkipSize         = (uint8_t)atoi(argv[1]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.rightSkipSize        = (uint8_t)atoi(argv[2]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.leftSkipSizeAzimuth  = (uint8_t)atoi(argv[3]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.rightSkipSizeAzimuth = (uint8_t)atoi(argv[4]);

    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.searchWinSizeRange   = (uint8_t)atoi(argv[5]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.searchWinSizeDoppler = (uint8_t)atoi(argv[6]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.searchWinSizeNear    = (uint8_t)atoi(argv[7]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.guardSizeRange       = (uint8_t)atoi(argv[8]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.guardSizeDoppler     = (uint8_t)atoi(argv[9]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.guardSizeNear        = (uint8_t)atoi(argv[10]);

    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.K0                     = (float)atof(argv[11]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.dopplerSearchRelThr    = (float)atof(argv[12]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicSideLobeThr                       = (float)atof(argv[13]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.enableSecondPassSearch = (uint8_t)atoi(argv[14]);
    gMmwMssMCB.dspPreStartCfgLocal.dynamicCfarConfig.rangeRefIndex          = (uint8_t)atoi(argv[15]);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for static scene RA CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIStaticRACfarCfg(int32_t argc, char *argv[])
{
    if (argc != (12 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.leftSkipSize           = (uint8_t)atoi(argv[1]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.rightSkipSize          = (uint8_t)atoi(argv[2]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.leftSkipSizeAzimuth    = (uint8_t)atoi(argv[3]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.rightSkipSizeAzimuth   = (uint8_t)atoi(argv[4]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.searchWinSizeRange     = (uint8_t)atoi(argv[5]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.searchWinSizeDoppler   = (uint8_t)atoi(argv[6]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.guardSizeRange         = (uint8_t)atoi(argv[7]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.guardSizeDoppler       = (uint8_t)atoi(argv[8]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.K0                     = (float)atof(argv[9]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.dopplerSearchRelThr    = (float)atof(argv[10]);
    gMmwMssMCB.dspPreStartCfgLocal.staticSideLobeThr                       = (float)atof(argv[11]);
    gMmwMssMCB.dspPreStartCfgLocal.staticCfarConfig.enableSecondPassSearch = (uint8_t)atoi(argv[12]);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for dynamic scene range-angle config
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIDynRngAngleCfg(int32_t argc, char *argv[])
{
    if (argc != (4 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.dspPreStartCfgLocal.rangeAngleCfg.searchStep       = (float)atof(argv[1]);
    gMmwMssMCB.dspPreStartCfgLocal.rangeAngleCfg.mvdr_alpha       = (float)atof(argv[2]);
    gMmwMssMCB.dspPreStartCfgLocal.rangeAngleCfg.detectionMethod  = (uint8_t)atoi(argv[3]);
    gMmwMssMCB.dspPreStartCfgLocal.rangeAngleCfg.dopplerEstMethod = (uint8_t)atoi(argv[4]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for dynamic scene 2D angle estimation config
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIDynAngleEstCfg(int32_t argc, char *argv[])
{

    if (gMmwMssMCB.dspPreStartCfgLocal.rangeAngleCfg.detectionMethod <= 1)
    {
        if (argc != (8+1))
        {
            CLI_write ("Error: Invalid usage of the CLI command\n");
            return -1;
        }
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.elevSearchStep  = (float)atof(argv[1]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.mvdr_alpha      = (float)atof(argv[2]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.maxNpeak2Search = (uint8_t)atoi(argv[3]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.peakExpSamples  = (uint8_t)atoi(argv[4]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.elevOnly        = (uint8_t)atoi(argv[5]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.sideLobThr      = (float)atof(argv[6]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.peakExpRelThr   = (float)atof(argv[7]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevAngleEstCfg.peakExpSNRThr   = (float)atof(argv[8]);
    }
    else
    {
        if (argc != (6+1))
        {
            CLI_write ("Error: Invalid usage of the CLI command\n");
            return -1;
        }
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevZoominCfg.zoominFactor      = (uint8_t)atoi(argv[1]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevZoominCfg.zoominNn8bors     = (uint8_t)atoi(argv[2]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevZoominCfg.peakExpSamples    = (uint8_t)atoi(argv[3]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevZoominCfg.peakExpRelThr     = (float)atof(argv[4]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevZoominCfg.peakExpSNRThr     = (float)atof(argv[5]);
        gMmwMssMCB.dspPreStartCfgLocal.angle2DEst.azimElevZoominCfg.localMaxCheckFlag = (uint8_t)atoi(argv[6]);
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for doppler bin selection config
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIDoppBinSelCfg(int32_t argc, char *argv[])
{
    if (argc != (4 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.dspPreStartCfgLocal.doppBiningCfg.doppBinSelEnable = (uint16_t)atoi(argv[1]);
    gMmwMssMCB.dspPreStartCfgLocal.doppBiningCfg.doppFFTSize      = (uint16_t)atoi(argv[2]);
    gMmwMssMCB.dspPreStartCfgLocal.doppBiningCfg.doppSelMinBin    = (uint16_t)atoi(argv[3]);
    gMmwMssMCB.dspPreStartCfgLocal.doppBiningCfg.doppSelMaxBin    = (uint16_t)atoi(argv[4]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Doppler Estimation configuration if the Doppler estimation method is CFAR
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIDopplerCFARCfg(int32_t argc, char *argv[])
{
    if (argc != (5 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if (gMmwMssMCB.dspPreStartCfgLocal.rangeAngleCfg.dopplerEstMethod == 1)
    {
        gMmwMssMCB.dspPreStartCfgLocal.dopCfarCfg.cfarDiscardLeft  = (uint8_t)atoi(argv[1]);
        gMmwMssMCB.dspPreStartCfgLocal.dopCfarCfg.cfarDiscardRight = (uint8_t)atoi(argv[2]);
        gMmwMssMCB.dspPreStartCfgLocal.dopCfarCfg.guardWinSize     = (uint8_t)atoi(argv[3]);
        gMmwMssMCB.dspPreStartCfgLocal.dopCfarCfg.refWinSize       = (uint8_t)atoi(argv[4]);
        gMmwMssMCB.dspPreStartCfgLocal.dopCfarCfg.thre             = (float)atof(argv[5]);
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for static scene range-angle config
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIStaticRngAngleCfg(int32_t argc, char *argv[])
{
    if (argc != (3 + 1))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.dspPreStartCfgLocal.staticEstCfg.staticProcEnabled        = (uint8_t)atoi(argv[1]);
    gMmwMssMCB.dspPreStartCfgLocal.staticEstCfg.staticAzimStepDeciFactor = (uint8_t)atoi(argv[2]);
    gMmwMssMCB.dspPreStartCfgLocal.staticEstCfg.staticElevStepDeciFactor = (uint8_t)atoi(argv[3]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for angle FOV config
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIAntAngleFoV(int32_t argc, char *argv[])
{
     if (argc != (2 + 1))
     {
         CLI_write ("Error: Invalid usage of the CLI command\n");
         return -1;
     }

    gMmwMssMCB.dspPreStartCfgLocal.fovCfg[0] = (float)atof(argv[1]);
    gMmwMssMCB.dspPreStartCfgLocal.fovCfg[1] = (float)atof(argv[2]);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for board antenna geometry matrix row 0
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIBoardAntGeometry0(int32_t argc, char *argv[])
{
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1 + SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    argInd = 1;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        gMmwMssMCB.dspPreStartCfgLocal.m_ind[i] = (int8_t)atoi(argv[argInd++]);
        gMmwMssMCB.antennaGeometryCfg.ant[i].col = -gMmwMssMCB.dspPreStartCfgLocal.m_ind[i];
    }
    gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts = 0;
    gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts = 0;
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for board antenna geometry matrix row 1
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIBoardAntGeometry1(int32_t argc, char *argv[])
{
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1 + SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    argInd = 1;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        gMmwMssMCB.dspPreStartCfgLocal.n_ind[i] = (int8_t)atoi(argv[argInd++]);
        gMmwMssMCB.antennaGeometryCfg.ant[i].row = -gMmwMssMCB.dspPreStartCfgLocal.n_ind[i];
    }
    gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts = 0;
    gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts = 0;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for board antenna phase rotation
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t mmwLab_CLIBoardAntPhaseRot(int32_t argc, char *argv[])
{
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1 + SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    argInd = 1;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        gMmwMssMCB.dspPreStartCfgLocal.phaseRot[i] = (int8_t)atoi(argv[argInd++]);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI for zone/cuboid population
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// static int32_t mmwLab_CLICuboidCfg(int32_t argc, char *argv[])
// {
//     uint32_t zoneInd;
//     uint32_t cuboidInd;
//     /* Sanity Check: */
//     if (argc != (1 + 8))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     zoneInd = atoi (argv[1]);
//     if (zoneInd < gMmwMssMCB.sbrCliCurrentZoneInd)
//     {
//         CLI_write("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

//     cuboidInd = gMmwMssMCB.sbrCliCurrentCuboidInd;

//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.cuboidDefs[cuboidInd].x1 = (float) atof (argv[3]);
//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.cuboidDefs[cuboidInd].x2 = (float) atof (argv[4]);
//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.cuboidDefs[cuboidInd].y1 = (float) atof (argv[5]);
//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.cuboidDefs[cuboidInd].y2 = (float) atof (argv[6]);
//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.cuboidDefs[cuboidInd].z1 = (float) atof (argv[7]);
//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.cuboidDefs[cuboidInd].z2 = (float) atof (argv[8]);

//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.numCuboidsPerOccupancyBox[zoneInd]++;

//     gMmwMssMCB.featureExtrModuleCfg.sceneryParams.numOccupancyBoxes = zoneInd + 1;
//     gMmwMssMCB.sbrCliCurrentZoneInd = zoneInd;
//     gMmwMssMCB.sbrCliCurrentCuboidInd = cuboidInd + 1;
//     return 0;
// }

/**
 *  @b Description
 *  @n
 *      This is the CLI for feature extraction configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// static int32_t mmwLab_CLIFeatExtrCfg(int32_t argc, char *argv[])
// {
//     /* Sanity Check: */
//     if (argc != (1 + 6))
//     {
//         CLI_write ("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }


//     gMmwMssMCB.featureExtrModuleCfg.maxNumPointsPerZonePerFrame = (uint16_t) atoi (argv[1]);
//     gMmwMssMCB.featureExtrModuleCfg.numFramesProc               = (uint16_t) atoi (argv[2]);
//     gMmwMssMCB.featureExtrModuleCfg.offsetCorrection            = (uint8_t) atoi (argv[3]);
//     gMmwMssMCB.featureExtrModuleCfg.dbScanFiltering             = (uint8_t) atoi (argv[4]);
//     gMmwMssMCB.featureExtrModuleCfg.dbScanEpsilon               = (float) atof (argv[5]);
//     gMmwMssMCB.featureExtrModuleCfg.dbScanMinPts                = (uint16_t) atoi (argv[6]);

//     return 0;
// }

/**
 *  @b Description
 *  @n
 *      This is the CLI for z-offset configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
// static int32_t mmwLab_CLIzOffsetCfg(int32_t argc, char *argv[])
// {
//     /* Sanity Check: */
//     if (argc != (1 + gMmwMssMCB.featureExtrModuleCfg.sceneryParams.numOccupancyBoxes))
//     {
//         CLI_write("Error: Invalid usage of the CLI command\n");
//         return -1;
//     }

    
//     uint8_t boxIdx;
//     for (boxIdx = 0; boxIdx < gMmwMssMCB.featureExtrModuleCfg.sceneryParams.numOccupancyBoxes; boxIdx++)
//     {
//         gMmwMssMCB.featureExtrModuleCfg.zOffset[boxIdx] = (float) atof (argv[boxIdx+1]);
//     }
//     gMmwMssMCB.cli_zOffsetCmdPending = 1;
//     return 0;
// }

//Enable this define, to allow dynamic swith to 6.25Mbps baud rate, then execute CLI cmd: baudRate 6250000
//#define ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG

#ifdef ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG
#include <drivers/uart.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/soc.h>

typedef struct {

    uint32_t moduleId;
    uint32_t clkId;
    uint32_t clkRate;

} SOC_ModuleClockFrequency;

extern UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES];
extern UART_Config gUartConfig[CONFIG_UART_NUM_INSTANCES];
void Drivers_uartInit();
extern SOC_ModuleClockFrequency gSocModulesClockFrequency[5];
extern void PowerClock_init(void);

static int32_t MmwDemo_CLIChangeUartBaudRate (int32_t argc, char* argv[])
{
    uint32_t baudRate;
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    baudRate = (uint32_t) atoi (argv[1]);

    if ((baudRate != 1250000) && (baudRate != 1562500) && (baudRate != 2500000) && (baudRate != 3125000) && (baudRate != 6250000))
    {
        CLI_write ("Error: Unsupported baud rate\n");
        return -1;
    }


    //change baud rate of data port
    gUartParams[1].baudRate = baudRate;

#if 1
    if ( baudRate > 1250000 )
    {
        gUartConfig[1].attrs->inputClkFreq = 200000000;

        Drivers_uartClose();
        UART_deinit();

        gSocModulesClockFrequency[3].clkId = SOC_RcmPeripheralClockSource_FAST_CLK1;
        gSocModulesClockFrequency[3].clkRate = 200000000;

#if 1
        PowerClock_init();
#else
        //Change the UART clock source
        uint32_t regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL); //Read register current value
        HW_SET_FIELD32(regVal,CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_SRCSEL ,7U); // 7U is for the source fast clock 1
        HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL, regVal);    //Write to register
#endif
    }
    else
    {
        gUartConfig[1].attrs->inputClkFreq = 40000000;

        Drivers_uartClose();
        UART_deinit();

        gSocModulesClockFrequency[3].clkId = SOC_RcmPeripheralClockSource_OSC_CLK;
        gSocModulesClockFrequency[3].clkRate = 40000000;

#if 1
        PowerClock_init();
#else
        //Change the UART clock source
        uint32_t regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL); //Read register current value
        HW_SET_FIELD32(regVal,CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_SRCSEL ,0U); // 0U is for the source clock 1
        HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_UART_1_CLKCTL, regVal);    //Write to register
#endif
    }
#else
    Drivers_uartClose();
    UART_deinit();
#endif
    Drivers_uartInit();
    Drivers_uartOpen();

    return 0;
}
#endif
/**************************************************************************************************/
/* Above is for Minor Motion - Capon                                                                   */
/**************************************************************************************************/

void CLI_write (const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;
    UART_Transaction trans;
    UART_Handle uartWriteHandle;

    uartWriteHandle = gCLI.cfg.UartHandle;

    UART_Transaction_init(&trans);

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* If CLI_write is called before CLI init has happened, return */
    if (uartWriteHandle == NULL)
    {
        return;
    }

    trans.buf   = &logMessage[0U];
    trans.count = sizeMessage;

    /* Log the message on the UART CLI console: */
    /* Blocking Mode: */
    UART_write (uartWriteHandle, &trans);
}

/* Spatial3D: DEPRECATED (Phase 1). The fall-aware tracker-driven cube gate this used to
 * configure was removed - the tracker freezes at fall, so a track-gated burst never fired
 * when it mattered. Cube extraction is now server-driven per RANGE BIN via `cubeQuery`
 * (see MmwDemo_CLICubeQuery). This handler is kept only so pre-existing .cfg files that
 * still carry a `trackBinCubeCfg ...` line load without a "not recognized" error; it does
 * nothing. */
int32_t MmwDemo_CLITrackBinCubeCfg (int32_t argc, char* argv[])
{
    (void)argc;
    (void)argv;
    return 0;   /* no-op: superseded by cubeQuery */
}

/* Spatial3D: server-triggered RANGE-WINDOW cube burst (the fall second-check).
 * cubeQuery <range_bin> <half_win> <n_frames>
 *   range_bin : center range bin to sample (from the fall's point-cloud range, NOT a track)
 *   half_win  : range bins each side of range_bin (2 = 5 bins, 3 = 7 bins)
 *   n_frames  : number of frames to burst TLV 320, after which extraction stops on its own
 * Arms gMmwMssMCB.tbcQuery* ; DPC_Execute picks it up on the next frame and emits TLV 320
 * for n_frames at that window, track-independent (a lost/frozen track cannot suppress it).
 * Arming order matters: window + frame count are written BEFORE the active flag so the DPC
 * task never observes an armed query with a stale frame count. */
int32_t MmwDemo_CLICubeQuery (int32_t argc, char* argv[])
{
    int32_t bin, half, nFrames;
    if (argc < 4)
    {
        CLI_write ("Error: cubeQuery <range_bin> <half_win> <n_frames>\n");
        return -1;
    }
    bin     = atoi (argv[1]);
    half    = atoi (argv[2]);
    nFrames = atoi (argv[3]);
    if (bin < 0 || half < 0 || nFrames <= 0)
    {
        CLI_write ("Error: cubeQuery needs range_bin>=0, half_win>=0, n_frames>0\n");
        return -1;
    }
    if (half > (TBC_MAX_ENTRIES - 1) / 2)   /* keep 2*half+1 within the entry buffer */
    {
        half = (TBC_MAX_ENTRIES - 1) / 2;
    }
    /* GUARD (cubeGuardCfg): clamp to the single-query hard cap AND the remaining budget in
     * the current window, so no host request can flood 320 and wedge the sensor. */
    if (nFrames > (int32_t) gMmwMssMCB.tbcMaxFramesPerQuery)
    {
        nFrames = (int32_t) gMmwMssMCB.tbcMaxFramesPerQuery;
    }
    {
        /* token bucket: whole tokens currently available (milli / 1000) */
        int32_t avail = gMmwMssMCB.tbcTokensMilli / 1000;
        if (avail <= 0)
        {
            CLI_write ("cubeQuery: no tokens (bucket empty)\n");
            return 0;                        /* refuse -- protect the frame pipeline */
        }
        if (nFrames > avail)
        {
            nFrames = avail;                 /* clamp to what the bucket can pay for now */
        }
    }
    gMmwMssMCB.tbcQueryBin        = (uint16_t) bin;
    gMmwMssMCB.tbcQueryHalfWin    = (uint16_t) half;
    gMmwMssMCB.tbcQueryFramesLeft = nFrames;
    gMmwMssMCB.tbcQueryActive     = 1;      /* arm last */
    return 0;
}

/**
 * cubeGuardCfg <maxFramesPerQuery> <budgetFrames> <budgetWindowFrames>
 *   Firmware self-protection for the 320 cube burst (see MMwMssMCB.tbc* guard fields).
 *   maxFramesPerQuery  : hard cap on a single cubeQuery (frames; 300 = 30 s @ 10 fps)
 *   budgetFrames       : max cube-frames allowed per window (300 = 30 s of cube)
 *   budgetWindowFrames : rolling window length (frames; 3000 = 300 s)
 *   Default 300 300 3000 = a single query <= 30 s, and <= 30 s of cube per any 300 s (10%).
 */
int32_t MmwDemo_CLICubeGuardCfg (int32_t argc, char* argv[])
{
    if (argc < 4)
    {
        CLI_write ("Error: cubeGuardCfg <maxPerQuery> <capacity> <refillWindowFrames>\n");
        return -1;
    }
    /* cubeGuardCfg <maxPerQuery> <capacity> <refillWindowFrames> -- token bucket (see the .h note).
     * refill/frame(milli) = capacity*1000 / refillWindow. Start the bucket FULL so a fall right
     * after boot is not throttled. */
    gMmwMssMCB.tbcMaxFramesPerQuery = (uint16_t) atoi (argv[1]);
    gMmwMssMCB.tbcBudgetFrames      = (uint16_t) atoi (argv[2]);     /* capacity (whole tokens) */
    gMmwMssMCB.tbcBudgetWindow      = (uint16_t) atoi (argv[3]);     /* frames to refill one bucket */
    if (gMmwMssMCB.tbcBudgetWindow < 1) { gMmwMssMCB.tbcBudgetWindow = 1; }
    gMmwMssMCB.tbcRefillMilli = ((int32_t)gMmwMssMCB.tbcBudgetFrames * 1000) /
                                (int32_t)gMmwMssMCB.tbcBudgetWindow;
    gMmwMssMCB.tbcTokensMilli = (int32_t)gMmwMssMCB.tbcBudgetFrames * 1000;  /* start full */
    gMmwMssMCB.tbcTokenHbCtr  = 0;
    return 0;
}

/**
 * poseCfg <enable> [zOffset_cm] [mount_cm] [tilt_deg] [margin_cm] [sustain] [elevAcc_deg]
 *   enable    : 1 run BOTH per-track fall legs each frame and emit TLV 321, 0 off
 *   zOffset_cm: MLP leg -- height remap (cm) added to posZ so a standing person
 *               reads TI's reference posz (~ +33 cm). Default 0.
 *   mount_cm  : window leg -- radar height above floor (cm). Default 100.
 *   tilt_deg  : window leg -- radar DOWN-tilt (deg, 0 = horizontal). Default 0.
 *               world height h = mount + z*cos(tilt) - y*sin(tilt).
 *   margin_cm : window leg -- "down" when the 2nd-highest point's h <= this (cm).
 *               Default 45.
 *   sustain   : window leg -- frames held low before latching down. Default 5.
 *   elevAcc_deg: window leg -- elevation ACCURACY (deg). Per point, h -= 0.5*rad(acc)*R
 *               (slant range R) so a far scattered-up lying body still reads "down"
 *               (range-growing margin: tight near, looser far). Default 6. 0 = flat z=0.
 * Resets the per-track state, so send before sensorStart (or to re-arm).
 * Set mount/tilt to match the rig (e.g. 200 cm / 25 deg per dashboard-z-calib).
 */
int32_t MmwDemo_CLIPoseCfg (int32_t argc, char* argv[])
{
    int32_t enable, sustain = 5;
    float   zOffCm = 0.0f, mountCm = 100.0f, tiltDeg = 0.0f, marginCm = 45.0f, elevAccDeg = 6.0f;
    if (argc < 2)
    {
        CLI_write ("Error: poseCfg <enable> [zOffset_cm] [mount_cm] [tilt_deg] [margin_cm] [sustain] [elevAcc_deg]\n");
        return -1;
    }
    enable = atoi (argv[1]);
    if (argc >= 3) zOffCm     = (float) atof (argv[2]);
    if (argc >= 4) mountCm    = (float) atof (argv[3]);
    if (argc >= 5) tiltDeg    = (float) atof (argv[4]);
    if (argc >= 6) marginCm   = (float) atof (argv[5]);
    if (argc >= 7) sustain    = atoi (argv[6]);
    if (argc >= 8) elevAccDeg = (float) atof (argv[7]);

    PoseMlp_init ();
    PoseMlp_setZOffset (zOffCm * 0.01f);                   /* cm -> m */
    PoseMlp_setWindowCfg (mountCm * 0.01f,                 /* cm -> m */
                          tiltDeg * 0.01745329252f,        /* deg -> rad */
                          marginCm * 0.01f,                /* cm -> m */
                          (uint8_t) sustain, (uint8_t) sustain,
                          elevAccDeg * 0.01745329252f);    /* deg -> rad; per-point floor-slope comp */
    gMmwMssMCB.poseEnable     = (enable != 0) ? 1 : 0;
    gMmwMssMCB.poseNumResults = 0;
    return 0;
}

void CLI_init (uint8_t taskPriority)
{
    int32_t          status;
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0],
                       "******************************************\r\n" \
                       "%s MMW Demo %02d.%02d.%02d.%02d\r\n"  \
                       "******************************************\r\n",
                        DEVICE_STRING,
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );


    status = SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Spatial3D cubeQuery GUARD defaults (active from boot, before any cubeGuardCfg): a
     * single query <= 30 s (300 frames) and <= 30 s of cube per rolling 300 s (10% duty).
     * Without this the zero-init would set the window to 0 (no protection) and the per-query
     * cap to 0 (no cube at all). Override at runtime with `cubeGuardCfg`. */
    /* token-bucket defaults: capacity 450 tokens, refill 450 over 4500 frames = 0.1/frame = 10%
     * average; single query <= 300 frames. Bucket starts FULL. cubeGuardCfg overrides at runtime. */
    gMmwMssMCB.tbcMaxFramesPerQuery = 300;
    gMmwMssMCB.tbcBudgetFrames      = 450;      /* capacity (whole tokens) */
    gMmwMssMCB.tbcBudgetWindow      = 4500;     /* frames to refill one full bucket */
    gMmwMssMCB.tbcRefillMilli       = (450 * 1000) / 4500;   /* = 100 milli/frame = 0.1 token */
    gMmwMssMCB.tbcTokensMilli       = 450 * 1000;            /* start full */
    gMmwMssMCB.tbcTokenHbCtr        = 0;

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.UartHandle                   = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = CLI_TASK_PRIORITY;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.overridePlatform             = false;
    cliCfg.overridePlatformString       = NULL;

    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "<FrameStopMode>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveSensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "channelCfg";
    cliCfg.tableEntry[cnt].helpString     = "<RxChCtrlBitMask> <TxChCtrlBitMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveChannelCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "chirpComnCfg";
    cliCfg.tableEntry[cnt].helpString     = "<DigOutputSampRate> <DigOutputBitsSel> <DfeFirSel> <NumOfAdcSamples> <ChirpTxMimoPatSel> <ChirpRampEndTime> <ChirpRxHpfSel>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveChirpCommonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "chirpTimingCfg";
    cliCfg.tableEntry[cnt].helpString     = "<ChirpIdleTime> <ChirpAdcSkipSamples> <ChirpTxStartTime> <ChirpRfFreqSlope> <ChirpRfFreqStart>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveChirpTimingCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "frameCfg";
    cliCfg.tableEntry[cnt].helpString     = "<NumOfChirpsInBurst> <NumOfChirpsAccum> <BurstPeriodicity> <NumOfBurstsInFrame> <FramePeriodicity> <NumOfFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveFrameCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<pointCloud> <rangeProfile> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo> <trackerInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "trackBinCubeCfg";
    cliCfg.tableEntry[cnt].helpString     = "DEPRECATED (superseded by cubeQuery), no-op";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLITrackBinCubeCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cubeQuery";
    cliCfg.tableEntry[cnt].helpString     = "<range_bin> <half_win> <n_frames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICubeQuery;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cubeGuardCfg";
    cliCfg.tableEntry[cnt].helpString     = "<maxFramesPerQuery> <budgetFrames> <budgetWindowFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICubeGuardCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "poseCfg";
    cliCfg.tableEntry[cnt].helpString     = "<enable> [zOffset_cm] [mount_cm] [tilt_deg] [margin_cm] [sustain]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIPoseCfg;
    cnt++;

    // cliCfg.tableEntry[cnt].cmd             = "sensorPosition";
    // cliCfg.tableEntry[cnt].helpString      = "<X - offset> <Y - Offset> <Z - Height> <Azimuth Tilt> <Elevation Tilt>";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLISensorPositionCfg;
    // cnt++;

    cliCfg.tableEntry[cnt].cmd            = "clutterRemoval";
    cliCfg.tableEntry[cnt].helpString     = "<0-disable, 1-enable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveClutterRemoval;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "lowPowerCfg";
    cliCfg.tableEntry[cnt].helpString     = "<LowPowerModeEnable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveLowPwrModeEnable;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "factoryCalibCfg";
    cliCfg.tableEntry[cnt].helpString     = "<save enable> <restore enable> <rxGain> <backoff0> <Flash offset>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveFactoryCalConfig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd             = "antGeometryCfg";
    cliCfg.tableEntry[cnt].helpString      = "<row0> <col0> <row1> <col1> <row2> <col2> <row3> <col3> <row4> <col4> <....> <row15> <col15> <antDistX (mm)> <antDistY (mm)>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryCfg;
    cnt++;

    /**********************************************************/
    /* For Minor Motion -  Capon:                                  */
    /**********************************************************/

    cliCfg.tableEntry[cnt].cmd           = "numFrmPerSlidingWindowMinor";
    cliCfg.tableEntry[cnt].helpString    = "<numFrmPerSlidingWindow>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLINumFrmPerSlidingWindowMinor;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "dynamicRACfarCfg";
    cliCfg.tableEntry[cnt].helpString    = "<leftSkipSize> <rightSkipSize> <leftSkipSizeAzimuth> <rightSkipSizeAngle> <searchWinSizeRange> <searchWinSizeAngle> <searchWinSizeNear> <guardSizeRange> <guardSizeAngle> <guardSizeNear> <threRange> <threAngle> <threSidelob> <enSecondPass> <rangeRefIndex>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIDynRACfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "staticRACfarCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <leftSkipSize> <rightSkipSize> <leftSkipSizeAzimuth> <rightSkipSizeAngle> <searchWinSizeRange> <searchWinSizeAngle> <guardSizeRange> <guardSizeAngle> <threRange> <threAngle> <threSidelob> <enSecondPass>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIStaticRACfarCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "dynamicRangeAngleCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <searchStep> <mvdr_alpha> <detectionMethod> <dopplerEstMethod>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIDynRngAngleCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "dynamic2DAngleCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <elevSearchStep> <mvdr_alpha> <maxNpeak2Search> <peakExpSamples> <elevOnly> <sideLobThr> <peakExpRelThr> <peakExpSNRThr>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIDynAngleEstCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "dopplerBinSelCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <doppBinSelEnable> <doppFFTSize> <doppBinSelMin> <doppBinSelMax>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIDoppBinSelCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "dopplerCfarCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <discardLeft> <discardRight> <guardWinSize> <refWinSize> <threshold> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIDopplerCFARCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "staticRangeAngleCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <staticProcEnabled> <staticAzimStepDeciFactor> <staticElevStepDeciFactor>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIStaticRngAngleCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "fovCfg";
    cliCfg.tableEntry[cnt].helpString    = "<subFrameIdx> <azimFoV> <elevFoV> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIAntAngleFoV;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "antGeometry0";
    cliCfg.tableEntry[cnt].helpString    = "<elem0> <elem1> <elem2> <elem3> <elem4> <elem5> <elem6> <elem7> <elem8>  <elem9> <elem10> <elem11> <elem12> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIBoardAntGeometry0;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "antGeometry1";
    cliCfg.tableEntry[cnt].helpString    = "<elem0> <elem1> <elem2> <elem3> <elem4> <elem5> <elem6> <elem7> <elem8>  <elem9> <elem10> <elem11> <elem12> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIBoardAntGeometry1;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "antPhaseRot";
    cliCfg.tableEntry[cnt].helpString    = "<elem0> <elem1> <elem2> <elem3> <elem4> <elem5> <elem6> <elem7> <elem8>  <elem9> <elem10> <elem11> <elem12> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = mmwLab_CLIBoardAntPhaseRot;
    cnt++;

    /***********************************************************/
    /*  Tracker Parameters for Tracker State Machine */
    /***********************************************************/

    cliCfg.tableEntry[cnt].cmd           = "trackingCfg";
    cliCfg.tableEntry[cnt].helpString    = "<enable> <paramSet> <numPoints> <numTracks> <maxDoppler> <framePeriod>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLITrackingCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "staticBoundaryBox";
    cliCfg.tableEntry[cnt].helpString    = "<X min> <X Max> <Y min> <Y max> <Z min> <Z max>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLIStaticBoundaryBoxCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "boundaryBox";
    cliCfg.tableEntry[cnt].helpString    = "<X min> <X Max> <Y min> <Y max> <Z min> <Z max>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLIBoundaryBoxCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "gatingParam"; // PC: 4 gating volume, Limits are set to 3m in length, 2m in width, 0 no limit in doppler
    cliCfg.tableEntry[cnt].helpString    = "<gating volume> <length> <width> <doppler>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLIGatingParamCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "stateParam"; // PC: 10 frames to activate, 5 to forget, 10 active to free, 1000 static to free, 5 exit to free, 6000 sleep to free
    cliCfg.tableEntry[cnt].helpString    = "<det2act> <det2free> <act2free> <stat2free> <exit2free> <sleep2free>"; // det2act, det2free, act2free, stat2free, exit2free, sleep2free
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLIStateParamCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "allocationParam"; // PC: 250 SNR, 0.1 minimal velocity, 5 points, 1m in distance, 2m/s in velocity
    cliCfg.tableEntry[cnt].helpString    = "<SNRs> <minimal velocity> <points> <in distance> <in velocity>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemo_CLIAllocationParamCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd           = "maxAcceleration";
    cliCfg.tableEntry[cnt].helpString    = "<max X acc.> <max Y acc.> <max Z acc.>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn = MmwDemoCLIMaxAccelerationParamCfg;
    cnt++;


/**********************************************************/


    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistance> <searchWin>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re04> <Im04> <....> <Re15> <Im15> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcDataSource";
    cliCfg.tableEntry[cnt].helpString     = "<0-DFP, 1-File> <fileName>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAdcDataSourceCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcLogging";
    cliCfg.tableEntry[cnt].helpString     = "<0-disable, 1-enableLVDS>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAdcLogging;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "<FrameTrigMode> <LoopBackEn> <FrameLivMonEn> <FrameTrigTimerVal>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveSensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "runtimeCalibCfg";
    cliCfg.tableEntry[cnt].helpString     = "<CLPC enable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveRuntimeCalConfig;
    cnt++;

    // cliCfg.tableEntry[cnt].cmd            = "macroDopplerCfg";
    // cliCfg.tableEntry[cnt].helpString     = "<enable> <delayLineLen>";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMacroDopplerCfg;
    // cnt++;

    // cliCfg.tableEntry[cnt].cmd            = "macroDopMapScaleCfg";
    // cliCfg.tableEntry[cnt].helpString     = "<scaleZone1> <scaleZone2> <scaleZone3> <scaleZone4> <scaleZone5>";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMacroDopplerMapScaleCfg;
    // cnt++;

    // cliCfg.tableEntry[cnt].cmd            = "macroDopNumVoxelCfg";
    // cliCfg.tableEntry[cnt].helpString     = "<numVoxelZone1> <numVoxelZone2> <numVoxelZone3> <numVoxelZone4> <numVoxelZone5>";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMacroDopplerNumVoxelCfg;
    // cnt++;

    // cliCfg.tableEntry[cnt].cmd            = "macroDopRngBinOffsetCfg";
    // cliCfg.tableEntry[cnt].helpString     = "<rngBinOffsZone1> <rngBinOffsZone2> <rngBinOffsZone3> <rngBinOffsZone4> <rngBinOffsZone5>";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMacroDopplerRngBinOffsCfg;
    // cnt++;

    // cliCfg.tableEntry[cnt].cmd            = "macroDopSteerDbgCfg";
    // cliCfg.tableEntry[cnt].helpString     = "<0-steerVecBsedOnZones 1-steerVecAtBoreSight> ";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIMacroDopplerSteerDbgCfg;
    // cnt++;

    // cliCfg.tableEntry[cnt].cmd            = "pointCloudGenDbgCfg";
    // cliCfg.tableEntry[cnt].helpString     = "<0-normalOperation 1-disablePointCloudGen> ";
    // cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIPointCloudGenerationDbgCfg;
    // cnt++;

#ifdef ENABLE_UART_HIGH_BAUD_RATE_DYNAMIC_CFG
    cliCfg.tableEntry[cnt].cmd             = "baudRate";
    cliCfg.tableEntry[cnt].helpString      = "<baudRate>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = MmwDemo_CLIChangeUartBaudRate;
    cnt++;
#endif

    // Major Motion

    cliCfg.tableEntry[cnt].cmd            = "cfarProcCfg";
    cliCfg.tableEntry[cnt].helpString     = "<procDirection> <averageMode> <winLen> <guardLen> <noiseDiv> <cyclicMode> <thresholdScale> <peakGroupingEn>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveCfarProcCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "cfarFovCfg";
    cliCfg.tableEntry[cnt].helpString     = "<procDirection> <min (meters or m/s)> <max (meters or m/s)>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveCfarFovCfg;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "aoaProcCfg";
    cliCfg.tableEntry[cnt].helpString     = "<azimuthFftSize> <elevationFftSize>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAoaProcCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "aoaFovCfg";
    cliCfg.tableEntry[cnt].helpString     = "<minAzimuthDeg> <maxAzimuthDeg> <minElevationDeg> <maxElevationDeg>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveAoaCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd             = "antGeometryBoard";
    cliCfg.tableEntry[cnt].helpString      = "<boardName>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryBoard;
    cnt++;
    
    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        DebugP_log ("Error: Unable to open the CLI\r\n");
        return;     
    }
}

int32_t CLI_open (CLI_Cfg* ptrCLICfg)
{
    uint32_t        index;

    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;

    /* Initialize the CLI MCB: */
    memset ((void*)&gCLI, 0, sizeof(CLI_MCB));

    /* Copy over the configuration: */
    memcpy ((void *)&gCLI.cfg, (void *)ptrCLICfg, sizeof(CLI_Cfg));

    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (gCLI.cfg.tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            gCLI.numCLICommands = gCLI.numCLICommands + 1;
        }
    }

    /* Do we have a CLI Prompt specified?  */
    if (gCLI.cfg.cliPrompt == NULL)
        gCLI.cfg.cliPrompt = "CLI:/>";

    /* The CLI provides a help command by default:
     * - Since we are adding this at the end of the table; a user of this module can also
     *   override this to provide its own implementation. */
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmd           = "help";
    gCLI.cfg.tableEntry[gCLI.numCLICommands].helpString    = NULL;
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmdHandlerFxn = CLI_help;

    /* Increment the number of CLI commands: */
    gCLI.numCLICommands++;

    gCliTask = xTaskCreateStatic( CLI_task,   /* Pointer to the function that implements the task. */
                                  "cli_task_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  CLI_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,              /* We are not using the task parameter. */
                                  ptrCLICfg->taskPriority,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gCliTskStack,  /* pointer to stack base */
                                  &gCliTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gCliTask != NULL);

    return 0;
}
