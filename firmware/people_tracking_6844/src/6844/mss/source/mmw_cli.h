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

#ifndef MMW_CLI_H
#define MMW_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <drivers/uart.h>
#include <drivers/soc.h>
#include <source/mmwave_demo_mss.h>
#include <FreeRTOS.h>
#include <task.h>
#include <common/syscommon.h>
/**************************************************************************
 ************************* CLI Module Definitions *************************
 **************************************************************************/

#define     CLI_MAX_CMD         100

#define     CLI_MAX_ARGS        40

/* These defines need to move to common files. */
#define MMWAVE_SDK_VERSION_BUILD  0
#define MMWAVE_SDK_VERSION_BUGFIX 0
#define MMWAVE_SDK_VERSION_MINOR  6
#define MMWAVE_SDK_VERSION_MAJOR  5
/* Set this to 1 to bypass the CLI task and send the pre-defined radarCmdString cli configuration */
#define CLI_BYPASS 0
/* Maximum radar cli commands to use the CLI_BYPASS feature */
#define MAX_RADAR_CMD 100

/* Number of CLI commands reserved for antenna geometry, range and phase compensation */
#define CLI_ANT_GEO_PHASE_COMP_CMD 4

/**************************************************************************
 ************************** CLI Data Structures ***************************
 **************************************************************************/


/**
 * @brief   CLI command handler:
 *
 *  @param[in]  argc
 *      Number of arguments
 *  @param[in]  argv
 *      Pointer to the arguments
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
typedef int32_t (*CLI_CmdHandler)(int32_t argc, char* argv[]);

typedef struct CLI_CmdTableEntry_t
{
    /**
     * @brief   Command string
     */
    char*               cmd;

    /**
     * @brief   CLI Command Help string
     */
    char*               helpString;

    /**
     * @brief   Command Handler to be executed
     */
    CLI_CmdHandler      cmdHandlerFxn;
}CLI_CmdTableEntry;

typedef struct CLI_Cfg_t
{
    /**
     * @brief   CLI Prompt string (if any to be displayed)
     */
    char*               cliPrompt;

    /**
     * @brief   Optional banner string if any to be displayed on startup of the CLI
     */
    char*               cliBanner;

    /**
     * @brief   UART Handle used by the CLI
     */
    UART_Handle         UartHandle;

    /**
     * @brief   The CLI has an mmWave extension which can be enabled by this
     * field. The extension supports the well define mmWave link CLI command(s)
     * In order to use the extension the application should have initialized
     * and setup the mmWave.
     */
    uint8_t             enableMMWaveExtension;

    /**
     * @brief   The mmWave control handle which needs to be specified if
     * the mmWave extensions are being used. The CLI Utility works only
     * in the FULL configuration mode. If the handle is opened in
     * MINIMAL configuration mode the CLI mmWave extension will fail
     */
    MMWave_Handle       mmWaveHandle;

    /**
     * @brief   Task Priority: The CLI executes in the context of a task
     * which executes with this priority
     */
    uint8_t             taskPriority;

    /**
     * @brief   Flag which determines if the CLI Write should use the UART
     * in polled or blocking mode.
     */
    bool                usePolledMode;

    /**
     * @brief   Flag which determines if the CLI should override the platform
     * string reported in @ref CLI_MMWaveVersion.
     */
    bool                overridePlatform;

    /**
     * @brief   Optional platform string to be used in @ref CLI_MMWaveVersion
     */
    char*               overridePlatformString;

    /**
     * @brief   This is the table which specifies the supported CLI commands
     */
    CLI_CmdTableEntry   tableEntry[CLI_MAX_CMD];
}CLI_Cfg;

typedef struct CLI_MCB_t
{
    /**
     * @brief   Configuration which was used to configure the CLI module
     */
    CLI_Cfg         cfg;

    /**
     * @brief   This is the number of CLI commands which have been added to the module
     */
    uint32_t        numCLICommands;

    /**
     * @brief   CLI Task Handle:
     */
    TaskHandle_t     cliTaskHandle;

    /**
     * @brief   CLI BYTask Semaphore Handle:
     */
    SemaphoreP_Object cliBypasssemaphoreObj;
}CLI_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
int32_t CLI_open (CLI_Cfg* ptrCLICfg);
void    CLI_write (const char* format, ...);
int32_t CLI_readLine(UART_Handle uartHandle, char *lineBuf, uint32_t bufSize);
int32_t CLI_close (void);
void    CLI_getMMWaveExtensionOpenConfig(MMWave_OpenCfg* ptrOpenCfg);
void    CLI_init (uint8_t taskPriority);

#ifdef __cplusplus
}
#endif

#endif /* MMW_CLI_H */
