/**
 * @file common_test.c
 *
 * @brief Source file containing definitions of APIs used in all tests.
 */
#include <source/common_test.h> /* spatial3d: was "common_test.h", not on -I when copied to project root */

void unityCharPut(uint8_t c)
{
    DebugP_log("%c", c);
    if (c == '\n')
    {
        DebugP_log("\r");
    }
}

void app_critSecStart(void)
{
    /* Empty - No RTOS */
}

void app_critSecStop(void)
{
    /* Empty - No RTOS */
}

/* spatial3d: app_ioWrite/app_ioRead removed - duplicated in mmwave_demo_mss.c; keep only critSec stubs to resolve app_critSecStart/Stop */
