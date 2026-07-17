/*----------------------------------------------------------------------------*/
/* r5f_linker.cmd                                                             */
/*                                                                            */
/* (c) Texas Instruments 2024, All rights reserved.                           */
/*----------------------------------------------------------------------------*/

/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=4096
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=4096

--retain="*(.irqStack)"
--retain="*(.fiqStack)"
--retain="*(.abortStack)"
--retain="*(.undStack)"
--retain="*(.svcStack)"

-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* Stack Sizes for various R5F modes */
__IRQ_STACK_SIZE = 256;
__FIQ_STACK_SIZE = 256;
__ABORT_STACK_SIZE = 256;
__UNDEFINED_STACK_SIZE = 256;
__SVC_STACK_SIZE = 4096;

#define MSS_L3_SIZE (0xB0000 + 0x70000)

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
MEMORY{
PAGE 0:
    RESET_VECTORS  (X)  : origin=0x00000000 length= 0x00000100                   /* Exception vectors */
    TCMA_RAM       (RX) : origin=0x00000100 length= (0x00080000 - 0x110)         /* TCMA RAM 512 KB in eclipsed mode */
    TCMB_RBL_Reserv(RW) : origin=0x08000000 length= 0x00009000                   /* TCMB RAM 36 KB used by RBL. Do not use for Code and Data Sections */
    TCMB_RAM       (RX) : origin=0x08009000 length= (0x00040000 - 0x9000)        /* TCMB RAM 256 KB */
    DSS_L3_MBOX    (RW) : origin=0x88000000 length= 0x400                        /* DSS L3 MBOX memory 1 KB */
    DSS_L3         (RW) : origin=0x88000400 length= MSS_L3_SIZE         			 /* DSS L3 used by MSS */
    DSS_L3_DSS     (RW) : origin=(0x88000400+MSS_L3_SIZE)  length= 0x160000 - (MSS_L3_SIZE + 0x400) /* DSS L3 used by DSS */
}

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > RESET_VECTORS

    /* This has the R5F boot code */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > TCMA_RAM

    /* Spatial3D: pin the ~51 KB pose MLP weights to TCMB. Placed BEFORE the
     * general .rodata group so first-match claims it here; the general group's
     * ">> TCMA_RAM | TCMB_RAM" spill must NOT get it, because TCMA has only
     * ~5.8 KB free after the cubeQuery patch and spilling 51 KB there bricks
     * boot (see fallsm-boot-bug). TCMB is 256 KB and already hosts FIX2. */
    .rodata.pose_model: {} palign(8) > TCMB_RAM

    GROUP {
        .text:   {} align(8)   /* This is where code resides */
        .rodata: {} align(8)   /* This is where const's go */
    } >> TCMA_RAM | TCMB_RAM

    GROUP {
        .data:   {} align(8)   /* This is where initialized globals and static go */
    } >> TCMA_RAM | TCMB_RAM

    GROUP {
        .bss:    {} align(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
    } >> TCMA_RAM | TCMB_RAM

    /* This is where the stacks for different R5F modes go */
    GROUP {
        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)
        RUN_START(__IRQ_STACK_START)
        RUN_END(__IRQ_STACK_END)
        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)
        RUN_START(__FIQ_STACK_START)
        RUN_END(__FIQ_STACK_END)
        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)
        RUN_START(__SVC_STACK_START)
        RUN_END(__SVC_STACK_END)
        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)
        RUN_START(__ABORT_STACK_START)
        RUN_END(__ABORT_STACK_END)
        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)
        RUN_START(__UNDEFINED_STACK_START)
        RUN_END(__UNDEFINED_STACK_END)
        .sysmem: {} align(8)   /* This is where the malloc heap goes */
        .stack:  {} align(8)   /* This is where the main() stack goes */
    } > TCMA_RAM

    /* Spatial3D FIX2: park the classifier task stack (gDspPointCloudTaskStack,
     * .bss.dsp_tcmb) in TCMB so TCMA keeps headroom - see fallsm-boot-bug. */
    .bss.dsp_tcmb {} align(32) > TCMB_RAM

    /* Spatial3D Phase 2: pose MLP runtime .bss (ring buffers + scratch, ~6.3 KB)
     * in TCMB for the same reason - TCMA has only ~5.8 KB free. */
    .bss.pose {} align(8) > TCMB_RAM

    /* any data buffer needed to be put in L3 can be assigned this section name */
    .bss.l3 {} > DSS_L3
}
/*----------------------------------------------------------------------------*/
