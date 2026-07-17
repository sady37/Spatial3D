#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hwa.h>
#include <drivers/hwa/v0/soc/hwa_soc.h>
#include <drivers/hw_include/hw_types.h>

#define HWA_GET_DRIVER_STRUCT(handle) \
    {\
        ptrHWADriver = (HWA_Driver *)handle;\
    }

int32_t HWA_loadParams(HWA_Handle handle, uint8_t paramsetStartIdx, uint8_t numParams, void * srcPtr)
{
    int32_t             retCode = 0;
    uint32_t *paramBasePtr;
    uint32_t *srcRegPtr = (uint32_t *) srcPtr;

    if (srcPtr == NULL)
    {
        retCode = HWA_EINVAL;
        goto exit;
    }

    /* Disable the HWA */
    retCode = HWA_enable(handle,0);
    if (retCode != 0)
    {
        goto exit;
    }

    paramBasePtr = (uint32_t *) HWA_getParamSetAddr(handle, paramsetStartIdx);

#if 0
    memcpy((void *) paramBasePtr, (void *) srcPtr, numParams * sizeof(DSSHWACCPARAMRegs));
#else
    for (int i = 0; i < (numParams * (sizeof(DSSHWACCPARAMRegs)/sizeof(uint32_t))); i++)
    {
        paramBasePtr[i] = srcRegPtr[i];
    }
#endif

exit:
    /* return */
    return retCode;
}

int32_t HWA_saveParams(HWA_Handle handle, uint8_t paramsetStartIdx, uint8_t numParams, void * dstPtr)
{
    int32_t             retCode = 0;
    uint32_t *paramBasePtr;
    uint32_t *dstRegPtr = (uint32_t *) dstPtr;

    if (dstPtr == NULL)
    {
        retCode = HWA_EINVAL;
        goto exit;
    }

    /* Disable the HWA */
    retCode = HWA_enable(handle,0);
    if (retCode != 0)
    {
        goto exit;
    }

    paramBasePtr = (uint32_t *) HWA_getParamSetAddr(handle, paramsetStartIdx);
#if 0
    memcpy((void *) dstPtr, (void *) paramBasePtr, numParams * sizeof(DSSHWACCPARAMRegs));
#else
    for (int i = 0; i < (numParams * (sizeof(DSSHWACCPARAMRegs)/sizeof(uint32_t))); i++)
    {
        dstRegPtr[i] = paramBasePtr[i];
    }
#endif

exit:
    /* return */
    return retCode;
}

