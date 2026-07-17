/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=2048
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=8192
--retain=_vectors

SECTIONS
{
    /* hard addresses forces vecs to be allocated there */
    .text:vectors: {. = align(1024); } > 0x00800000
    .text:      {} > DSS_L2
    .const:     {} > DSS_L2
    .cinit:     {} > DSS_L2
    .data:      {} > DSS_L2
    .stack:     {} > DSS_L2
    .switch:    {} > DSS_L2
    .cio:       {} > DSS_L2
    .sysmem:    {} > DSS_L2
    .fardata:   {} > DSS_L2
    .far:       {} > DSS_L3

    /* These should be grouped together to avoid STATIC_BASE relative relocation linker error */
    GROUP {
        .rodata:    {}
        .bss:       {}
        .neardata:  {}
    } > DSS_L2

    /* Sections needed for C++ projects */
    GROUP {
        .c6xabi.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array:    {} palign(8)   /* Contains function pointers called before main */
        .fini_array:    {} palign(8)   /* Contains function pointers called after main */
    } > DSS_L2

    /* any data buffer needed to be put in L3 can be assigned this section name */
    .bss.dss_l3 {} > DSS_L3

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM

	.ddrHeap: 		{} >> DSS_L3
	.L2heap: 		{} >> DSS_L2
	.L2ScratchSect: {} >> DSS_L3
	.L1ScratchSect: {} >> DSS_L1D
	.L1heap: 		{} >> DSS_L1D
	.ovly 			{} >  DSS_L2
#if 0
	.fastCode:
    { // NOTE: The following is experimental and should only be used when loading through the CCS debugger with the debug configuration 
		//DSPF_sp_fftSPxSP.obj (.text:optimized)
		RADARDEMO_detectionCFAR_priv.obj (.text:RADARDEMO_detectionCFAR_raCAAll)
	    RADARDEMO_aoaEst2DCaponBF_heatmapEst.obj (.text:RADARDEMO_aoaEst2DCaponBF_raHeatmap)
		//RADARDEMO_aoaEst2DCaponBF_angleEst.obj (.text:RADARDEMO_aoaEst2DCaponBF_aeEstElevOnly)
		RADARDEMO_aoaEst2DCaponBF_rnEstInv.obj (.text:RADARDEMO_aoaEst2DCaponBF_covInv)
		//RADARDEMO_aoaEst2DCaponBF.obj (.text:RADARDEMO_aoaEst2DCaponBF_run)
		MATRIX_cholesky.obj (.text:MATRIX_cholesky_flp_inv)
		//RADARDEMO_aoaEst2DCaponBF.obj (.text:RADARDEMO_aoaEst2DCaponBF_static_run)
		//RADARDEMO_aoaEst2DCaponBF_DopplerEst.obj (.text:RADARDEMO_aoaEst2DCaponBF_dopperEstInput)
		//RADARDEMO_detectionCFAR.obj (.text:RADARDEMO_detectionCFAR_run)
		//radarOsal_malloc.obj (.text:radarOsal_memAlloc)
		RADARDEMO_aoaEst2DCaponBF_staticRemoval.obj (.text:RADARDEMO_aoaEst2DCaponBF_clutterRemoval)
		copyTranspose.obj (.text:copyTranspose)

		//RADARDEMO_aoaEst2DCaponBF_staticHeatMapEst.obj (.text:RADARDEMO_aoaEstimationBFSinglePeak_static)
		//radarOsal_malloc.obj (.text:radarOsal_memInit)
		/*Note: memDeInit and memFree are not currently used. Will be added with Capon support. */
    	//radarOsal_malloc.obj (.text:radarOsal_memDeInit)
    	//radarOsal_malloc.obj (.text:radarOsal_memFree)
		//testAoaEst2DCaponBF.obj (.text:edmaTranspose)
  		dsplib.ae66 <DSPF_sp_fftSPxSP.oe66>(.text  DSPF_sp_fftSPxSP)
    } load=DSS_L3_TEXT  , run=DSS_L1P , table(_mmwDemo_fastCode_DSS_L1P_copy_table, compression=off)
#endif

}
#define MSS_L3RAM_SIZE (0xB0000 + 0x70000)
#define L1P_CACHE_SIZE (32*1024)
#define L1D_CACHE_SIZE (16*1024)

MEMORY
{
	//DSS_L1P:      ORIGIN = 0xE00000, LENGTH = 0x00008000-L1P_CACHE_SIZE
	DSS_L1D:      ORIGIN = 0xF00000, LENGTH = 0x00008000-L1D_CACHE_SIZE
    DSS_L2:       ORIGIN = 0x800000, LENGTH = 0x60000
    DSS_L3_IPC:   ORIGIN = 0x88000000, LENGTH = 0x00000400
    DSS_L3_MSS:   ORIGIN = 0x88000400, LENGTH = MSS_L3RAM_SIZE
    DSS_L3:       ORIGIN = (0x88000400 + MSS_L3RAM_SIZE), LENGTH = 0x160000 - (MSS_L3RAM_SIZE + 0x400)
//  DSS_L3:       ORIGIN = 0x880B0400, LENGTH = (MSS_L3RAM_SIZE - 0x400 - 0x4000)


    /* shared memories that are used by RTOS/NORTOS cores */
    /* On C66,
     * - make sure these are which mapped as non-cache in MAR bits
     */
    USER_SHM_MEM            : ORIGIN = 0xC02E8000, LENGTH = 0x00004000
    LOG_SHM_MEM             : ORIGIN = 0xC02EC000, LENGTH = 0x00004000
}
