#ifndef HWA_ADAPTATION
#define HWA_ADAPTATION
    
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hwa.h>
#include <drivers/hwa/v0/soc/hwa_soc.h>
#include <drivers/hw_include/hw_types.h>

#define HWA_NUM_REG_PER_PARAM_SET   8

int32_t HWA_loadParams(HWA_Handle handle, uint8_t paramsetStartIdx, uint8_t numParams, void * srcPtr);
int32_t HWA_saveParams(HWA_Handle handle, uint8_t paramsetStartIdx, uint8_t numParams, void * dstPtr);


#ifdef __cplusplus
}
#endif

#endif /* HWA_ADAPTATION */
