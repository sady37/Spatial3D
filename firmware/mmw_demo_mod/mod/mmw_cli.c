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
#include <source/mmw_res.h>
#include <source/mmwave_demo.h>
#include <source/calibrations/factory_cal.h>

#include <drivers/uart.h>
#include <drivers/prcm.h>
#include <drivers/pinmux.h>
#include <drivers/mcspi/v0/dma/mcspi_dma.h>
#include <drivers/mcspi/v0/dma/edma/mcspi_dma_edma.h>
#include <drivers/gpio.h>
#include <drivers/edma.h>
#include <drivers/mcspi.h>
#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>

#include <common/syscommon.h>

#include <utils/mathutils/mathutils.h>
#include <datapath/dpif/dpif_adcdata.h>
#include <datapath/dpif/dpif_chcomp.h>
#include <datapath/dpu/cfarproc/v1/cfarprochwa.h>

#include <ti_drivers_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <ti_board_config.h>

/* Device string used to print in the CLI banner */
#if defined (SOC_XWRL684X)
#define DEVICE_STRING "xWRL684x"
#endif

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

/* CLI commands designated for antenna geometry, range and phase compensation, these are specific to the xWRL6844EVM. For other devices, ensure a different global variable is used accordingly */
char* GantGeoRangePhaseCompxWRL6844EVM[CLI_ANT_GEO_PHASE_COMP_CMD] = {
    "antGeometryTX 2 2 2 0 0 0 0 2 \r\n",
    "antGeometryRx 1 0 0 0 0 1 1 1 \r\n",
    "antGeometryDist 2.54 2.54 \r\n",
    "compRangeBiasAndRxChanPhase 0.0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 -1 0 -1 0 -1 0 -1 0 1 0 1 0 1 0 1 0 \r\n"
};

CLI_MCB     gCLI;

TaskHandle_t gCliTask;
StaticTask_t gCliTaskObj;
StackType_t  gCliTskStack[CLI_TASK_STACK_SIZE] __attribute__((aligned(32)));

static SemaphoreP_Object gUartReadDoneSem;

#if (CLI_BYPASS == 1)
/* When CLI-BYPASS is enabled, CLI configurations specified in this structure are used */
char* GRadarCmdString[MAX_RADAR_CMD] =
{
	"sensorStop 0 \n\r",
	"channelCfg 153 255 0 \n\r",
	"chirpComnCfg 8 0 0 256 1 13.1 3 \n\r",
	"chirpTimingCfg 6 63 0 160 58 \n\r",
	"frameCfg 64 0 1358 1 100 0 \n\r",
	"guiMonitor 1 1 0 0 0 1 \n\r",
	"cfarProcCfg 0 2 8 4 3 0 9.0 0 \n\r",
	"cfarProcCfg 1 2 4 2 2 1 9.0 0 \n\r",
	"cfarFovCfg 0 0.25 9.0 \n\r",
	"cfarFovCfg 1 -20.16 20.16 \n\r",
	"aoaProcCfg 64 64 \n\r",
    "aoaFovCfg -60 60 -60 60 \n\r",
    "clutterRemoval 0 \n\r",
    "factoryCalibCfg 1 0 44 2 0x1ff000 \n\r",
    "runtimeCalibCfg 1 \n\r",
    "antGeometryBoard xWRL6844EVM \n\r",
    "adcDataSource 0 adc_test_data_0001.bin \n\r",
    "adcLogging 0 \n\r",
    "lowPowerCfg 1 \n\r",
    "sensorStart 0 0 0 0 \n\r",
};
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
static int32_t CLI_MMWaveGuiMonSel(int32_t argc, char* argv[]);
static int32_t CLI_MMWaveCfarProcCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveCfarFovCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAoaProcCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAoaCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveClutterRemoval (int32_t argc, char* argv[]); 
static int32_t CLI_MMWaveLowPwrModeEnable(int32_t argc, char* argv[]);
static int32_t CLI_MMWaveFactoryCalConfig (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveRuntimeCalConfig (int32_t argc, char* argv[]);
static int32_t CLI_MmwDemo_AntGeometryCfg (int32_t argc, char* argv[]);
static int32_t CLI_MmwDemo_AntGeometryBoard (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t CLI_MmwWaveGpAdcMeasConfig(int32_t argc, char* argv[]);
static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveADCDataDitherCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdcDataSourceCfg (int32_t argc, char* argv[]);
static int32_t CLI_MMWaveAdcLogging (int32_t argc, char* argv[]);
int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[]);

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern uint32_t gSensorStop;
extern MmwDemo_MSS_MCB gMmwMssMCB;

extern void MmwStart(void);

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
            CLI_write ("Skipped\n");
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
    char*                   tokenizedArgs[CLI_MAX_ARGS];
    char*                   ptrCLICommand;
    char                    delimitter[] = " \r\n";
    uint32_t                argIndex;
    CLI_CmdTableEntry*      ptrCLICommandEntry;
    int32_t                 cliStatus;
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
                if (cliStatus != 0)
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
    /* Resetting the range bias phase compensation, and antenna geometry variables to enable parsing of new CLI configurations. */
    GIsAntGeoDef = 1;
    GIsRangePhaseCompDef = 0;

    /*Setting the FTDI HOST INTR pin to default high state for proper behaviour of SPI ADC Streaming*/
    GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

    return 0;
}

static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[])
{
    uint32_t i;
    uint32_t index=0;

    
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    gMmwMssMCB.mmWaveCfg.rxEnbl  = atoi (argv[1]);
    gMmwMssMCB.mmWaveCfg.txEnbl  = atoi (argv[2]);

    gMmwMssMCB.numRxAntennas = 0;
    gMmwMssMCB.numTxAntennas = 0;
    for (i = 1; i < (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL); i+=2)
    {
        /*At a time only a or b bitmasks are effectively used. In 6844, it is additively combined, still giving only one effective channel*/
        if(((gMmwMssMCB.mmWaveCfg.txEnbl >> i) & 0x1) || ((gMmwMssMCB.mmWaveCfg.txEnbl >> (i-1)) & 0x1))
        {
            if (gMmwMssMCB.numTxAntennas < SYS_COMMON_NUM_TX_ANTENNAS)
            {
                gMmwMssMCB.txAntOrder[gMmwMssMCB.numTxAntennas] = (uint8_t) index;
                gMmwMssMCB.numTxAntennas++;
            }
            else
            {
                CLI_write ("Error: Number of selected Tx antennas must be less than or equal to max Tx\n");
                return -1;
            }
        }
        
        /*At a time only a or b bitmasks are effectively used*/
        if(((gMmwMssMCB.mmWaveCfg.rxEnbl >> i) & 0x1) || ((gMmwMssMCB.mmWaveCfg.rxEnbl >> (i-1)) & 0x1))
        {
            if (gMmwMssMCB.numRxAntennas < SYS_COMMON_NUM_RX_CHANNEL)
            {
                gMmwMssMCB.rxAntOrder[gMmwMssMCB.numRxAntennas] = (uint8_t) index;
                gMmwMssMCB.numRxAntennas++;
            }
            else
            {
                CLI_write ("Error: Number of selected Rx antennas must be less than or equal to max Rx\n");
                return -1;
            }  
        }
        index++;
    }

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
        gMmwMssMCB.numRangeBins = mathUtils_pow2roundup(gMmwMssMCB.mmWaveCfg.profileComCfg.numOfAdcSamples)/2; //Real only sampling
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
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpAdcStartTime  = atoi (argv[2]);
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpTxStartTimeus = atof (argv[3]);
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpSlope         = atof (argv[4]); 
    gMmwMssMCB.mmWaveCfg.profileTimeCfg.startFreqGHz       = atof (argv[5]); 

    if(gMmwMssMCB.mmWaveCfg.profileTimeCfg.chirpAdcStartTime > MAX_CHIRP_ADC_SKIP_SAMPLES)
    {
        CLI_write("Error: Number of chirp ADC skip samples exceeds the maximum supported limit of %d.\n", MAX_CHIRP_ADC_SKIP_SAMPLES);
        return -1;
    }
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
    gMmwMssMCB.mmWaveCfg.frameCfg.framePeriodicityus      = (atof (argv[5])*1000);
    gMmwMssMCB.mmWaveCfg.frameCfg.numOfFrames             = atoi (argv[6]);

    if ((gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst * gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame)%gMmwMssMCB.numTxAntennas != 0)
    {
        CLI_write("Error: Total number of chirps should be a multiple of number of Tx antennas.\n");
        return -1;
    }

    return 0;
}

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

    return 0;
}

/**
 *  Spatial3D: configure the range-antenna zero-Doppler complex TLV (type 8).
 *  Usage: rangeAntennaOutput <startBin> <numBins> <enable>
 *    startBin : first range bin to export
 *    numBins  : number of range bins (0 disables)
 *    enable   : 1 to emit the TLV, 0 to suppress (reuses rangeAzimuthHeatMap gate)
 */
static int32_t CLI_MMWaveRangeAntennaOutput (int32_t argc, char* argv[])
{
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.guiMonSel.rangeAntennaStartBin = (uint16_t) atoi (argv[1]);
    gMmwMssMCB.guiMonSel.rangeAntennaNumBins  = (uint16_t) atoi (argv[2]);
    gMmwMssMCB.guiMonSel.rangeAzimuthHeatMap  = (uint8_t)  atoi (argv[3]);

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

    /* Populate configuration: */
    gMmwMssMCB.fovAoaCfg.minAzimuthDeg      = (float) atof (argv[1]);
    gMmwMssMCB.fovAoaCfg.maxAzimuthDeg      = (float) atof (argv[2]);
    gMmwMssMCB.fovAoaCfg.minElevationDeg    = (float) atof (argv[3]);
    gMmwMssMCB.fovAoaCfg.maxElevationDeg    = (float) atof (argv[4]);
    
    return 0;
}

static int32_t CLI_MMWaveClutterRemoval (int32_t argc, char* argv[])
{
    uint16_t    numDopplerChirps, numDopplerBins;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.staticClutterRemovalEnable          = (bool) atoi (argv[1]);

    numDopplerChirps = (gMmwMssMCB.mmWaveCfg.frameCfg.numOfBurstsInFrame * gMmwMssMCB.mmWaveCfg.frameCfg.numOfChirpsInBurst)/gMmwMssMCB.numTxAntennas;
    numDopplerBins = mathUtils_pow2roundup(numDopplerChirps);

    if (gMmwMssMCB.staticClutterRemovalEnable && (numDopplerChirps != numDopplerBins))
    {
        CLI_write("Error: Total number doppler chirps must be a power of 2 when clutter removal is enabled.\n");
        return -1;
    }
    
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

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for antenna geometry configuration
 *      Arguments are row/column coordinates of TX, RX and antenna spacing in 
 *      units of lambda/2 for antGeometryTX, antGeometryRX, antGeometryDist respectively. The arguments are: 
 *      <row0> <col0> <row1> <col1> <row2> <col2> <row3> <col3> for antGeometryTX
 *      where 
 *           row0,col0 corresponds to tx0,
 *           row1,col1 corresponds to tx1,
 *           row2,col2 corresponds to tx2,
 *           row3,col3 corresponds to tx3
 *      <row0> <col0> <row1> <col1> <row2> <col2> <row3> <col3> for antGeometryRX
 *      where 
 *           row0,col0 corresponds to rx0,
 *           row1,col1 corresponds to rx1,
 *           row2,col2 corresponds to rx2,
 *           row3,col3 corresponds to rx3
 *      <antennaDistanceXdim> <antennaDistanceZdim> for antGeometryDist
 *         <antennaDistanceXdim> antenna spacing in X dimension in mm
 *         <antennaDistanceZdim> antenna spacing in Z dimension in mm
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
    int32_t txInd,
            rxInd,
            antInd,
            argInd;

    if ((argc != 9) && (argc != 3))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    if((GIsAntGeoDef >> 3) == 1)
    {
        CLI_write ("Error: Antenna geometry is already defined\n");
        return -1;
    }
    /* A value of 1000b in GIsAntGeoDef indicates its fully defined */
    GIsAntGeoDef = GIsAntGeoDef << 1;

    argInd = 1;
    if(strcmp(argv[0], "antGeometryTX") == 0)
    {
        for(txInd = 0; txInd < 2*SYS_COMMON_NUM_TX_ANTENNAS; txInd++)
        {
            GAntGeometryTX[txInd] = (int8_t) atoi(argv[argInd++]);
            if(GAntGeometryTX[txInd] < 0)
            {
                CLI_write ("Error: All antenna indices must be non-negative\n");
            }
        }
    }
    else if(strcmp(argv[0], "antGeometryRX") == 0)
    {
        for(rxInd = 0; rxInd < 2*SYS_COMMON_NUM_RX_CHANNEL; rxInd++)
        {
            GAntGeometryRX[rxInd] = (int8_t) atoi(argv[argInd++]);
            if(GAntGeometryRX[rxInd] < 0)
            {
                CLI_write ("Error: All antenna indices must be non-negative\n");
            }
        }
    }
    else
    {
        gMmwMssMCB.antennaGeometryCfg.antDistanceXdimMts = atof(argv[1]) * 1e-3;
        gMmwMssMCB.antennaGeometryCfg.antDistanceZdimMts = atof(argv[2]) * 1e-3;
    }

    if((GIsAntGeoDef >> 3) == 1)
    {
        antInd = 0;
        for(txInd = 0; txInd < 2*SYS_COMMON_NUM_TX_ANTENNAS; txInd+=2)
        {
            for(rxInd = 0; rxInd < 2*SYS_COMMON_NUM_RX_CHANNEL; rxInd+=2)
            {
                gMmwMssMCB.antennaGeometryCfg.ant[antInd].row = GAntGeometryTX[txInd] + GAntGeometryRX[rxInd];
                gMmwMssMCB.antennaGeometryCfg.ant[antInd].col = GAntGeometryTX[txInd + 1] + GAntGeometryRX[rxInd + 1];
                antInd++;
            }
        }
    }

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

        gMmwMssMCB.compRxChannelBiasCfg.rangeBias = rangePhaseCfg.rangeBias;

        for(txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
        {
            for(rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
            {
                gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[txInd * gMmwMssMCB.numRxAntennas + rxInd] =
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
static int32_t CLI_MMWaveMeasureRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
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
 *      This is the CLI command Handler for gpADC measurement config.
 *
 *  @param[in]  argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t CLI_MmwWaveGpAdcMeasConfig (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 3)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.gpAdcCfg.channelEnable = atoi (argv[1]);
    gMmwMssMCB.gpAdcCfg.volPrintsEnable = atoi (argv[2]);

    return 0;
}

static int32_t CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    DPU_AoAProc_compRxChannelBiasCfg   cfg;
    int32_t Re, Im;
    int32_t argInd;
    int32_t txInd,rxInd,antInd;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    if(GIsRangePhaseCompDef == 1)
    {
        CLI_write ("Error: Range bias and phase compensation is already defined\n");
        return -1;
    }

    /* range bias and phase compensation is fully defined */
    GIsRangePhaseCompDef  = 1;

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (antInd=0; antInd < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; antInd++)
    {
        Re = (int32_t) (atof (argv[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Re);
        cfg.rxChPhaseComp[antInd].real = (int16_t) Re;

        Im = (int32_t) (atof (argv[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Im);
        cfg.rxChPhaseComp[antInd].imag = (int16_t) Im;
    }
    
    gMmwMssMCB.compRxChannelBiasCfg.rangeBias = cfg.rangeBias;

    for(txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for(rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[txInd * gMmwMssMCB.numRxAntennas + rxInd] =
                cfg.rxChPhaseComp[gMmwMssMCB.txAntOrder[txInd] * SYS_COMMON_NUM_RX_CHANNEL +
                                                                gMmwMssMCB.rxAntOrder[rxInd]];
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for ADC data dithering configuration
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
static int32_t CLI_MMWaveADCDataDitherCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.adcDataDithEnable = (bool) atoi (argv[1]);

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
    int32_t errCode = SystemP_SUCCESS;

    /* Sanity Check: Minimum argument check */
    if ((argc != 2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.adcLogging.enable = (uint8_t) atoi (argv[1]);

    if (gMmwMssMCB.adcLogging.enable  == ADC_DATA_LOGGING_LVDS_STREAMING)
    {
        /* Initialize LVDS streaming components */
        if ((errCode = MmwDemo_LVDSStreamInit()) < 0 )
        {
            CLI_write("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n",errCode);
            MmwDemo_debugAssert (0);
        }
    }
    else
    {
        mmWDemo_parkLvdsPins();
    }
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
    if(((GIsAntGeoDef >> 3) != 1) || (GIsRangePhaseCompDef != 1))
    {
        CLI_write ("Error: Antenna geometry is not fully defined\n");
        return -1;
    }

    /* Populate the SensorStop configuration: */
    gMmwMssMCB.mmWaveCfg.strtCfg.frameTrigMode      = atoi (argv[1]);
    gMmwMssMCB.mmWaveCfg.strtCfg.chirpStartSigLbEn  = atoi (argv[2]);
    gMmwMssMCB.mmWaveCfg.strtCfg.frameLivMonEn      = atoi (argv[3]);
    gMmwMssMCB.mmWaveCfg.strtCfg.frameTrigTimerVal  = atoi (argv[4]);

    MmwStart();

    return 0;
}

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
    cliCfg.tableEntry[cnt].helpString     = "<RxChCtrlBitMask> <TxChCtrlBitMask> <Reserved>";
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
    cliCfg.tableEntry[cnt].helpString     = "<pointCloud> <rangeProfile> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "rangeAntennaOutput";
    cliCfg.tableEntry[cnt].helpString     = "<startBin> <numBins> <enable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveRangeAntennaOutput;
    cnt++;

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

    cliCfg.tableEntry[cnt].cmd            = "runtimeCalibCfg";
    cliCfg.tableEntry[cnt].helpString     = "<CLPC enable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveRuntimeCalConfig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd             = "antGeometryTX";
    cliCfg.tableEntry[cnt].helpString      = "<row0> <col0> <row1> <col1> <row2> <col2> <row3> <col3>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd             = "antGeometryRX";
    cliCfg.tableEntry[cnt].helpString      = "<row0> <col0> <row1> <col1> <row2> <col2> <row3> <col3>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd             = "antGeometryDist";
    cliCfg.tableEntry[cnt].helpString      = "<antDistX (mm)> <antDistY (mm)>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd             = "antGeometryBoard";
    cliCfg.tableEntry[cnt].helpString      = "<boardName>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn   = CLI_MmwDemo_AntGeometryBoard;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "measureRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<enabled> <targetDistanceMts> <searchWinSizeMts>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveMeasureRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "gpAdcMeasConfig";
    cliCfg.tableEntry[cnt].helpString     = "<channelEnable> <volPrintsEnable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MmwWaveGpAdcMeasConfig;
    cnt++;
    
    cliCfg.tableEntry[cnt].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[cnt].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re04> <Im04> <....> <Re15> <Im15> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveCompRangeBiasAndRxChanPhaseCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcDataDitherCfg";
    cliCfg.tableEntry[cnt].helpString     = "<adcDataDithEnable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = CLI_MMWaveADCDataDitherCfg;
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