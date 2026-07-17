/**
 *   @file  dpif_mss_dss.h
 *
 *   @brief
 *      mss dss api common header File
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2024 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DPIF_MSS_DSS_H
#define DPIF_MSS_DSS_H

#define DPIF_MAX_DYNAMIC_CFAR_PNTS          (200*2)
#define DPIF_MAX_STATIC_CFAR_PNTS           (200*2)
#define DPIF_DOA_OUTPUT_MAXPOINTS           (DPIF_MAX_DYNAMIC_CFAR_PNTS * 4 + DPIF_MAX_STATIC_CFAR_PNTS)
#define DPIF_MAX_RESOLVED_OBJECTS_PER_FRAME DPIF_DOA_OUTPUT_MAXPOINTS

/* mmWave SDK driver/common Include Files */
#include <drivers/hw_include/hw_types.h>
#include <drivers/soc.h>
#ifdef SUBSYS_MSS
#include <kernel/dpl/CacheP.h>
#endif


#include <common/syscommon.h>
#include <drivers/hw_include/csl_complex_math_types.h>


/* MMWAVE Driver Include Files */
#include <common/mmwave_error.h>
#include <datapath/dpif/dpif_pointcloud.h>
#include <datapath/dpif/dpif_radarcube.h>
#include <datapath/dpif/dpif_detmatrix.h>


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    DPIF_RADARDEMO_DETECTIONCFAR_CFAROS = 0, /**< CFAR type: ordered statistics*/
    DPIF_RADARDEMO_DETECTIONCFAR_CAVGCFAR, /**< CFAR type: cell average CFAR*/
    DPIF_RADARDEMO_DETECTIONCFAR_CASOCFAR, /**< CFAR type: cell average CFAR, smaller of the 2 windows*/
    DPIF_RADARDEMO_DETECTIONCFAR_CACCCFAR, /**< CFAR type: cell accumulation CFAR*/
    DPIF_RADARDEMO_DETECTIONCFAR_CAGOCFAR, /**< CFAR type: cell average CFAR, greater of the 2 windows*/
    DPIF_RADARDEMO_DETECTIONCFAR_RA_CASOCFAR, /**< CFAR type: cell average CFAR, smaller of the 2 windows for both 2 passes, and for range-azimuth*/
    DPIF_RADARDEMO_DETECTIONCFAR_RA_CASOCFARV2, /**< CFAR type: cell average CFAR, smaller of the 2 windows for range pass, angle pass local max search only, and for range-azimuth*/
    DPIF_RADARDEMO_DETECTIONCFAR_NOT_SUPPORTED,
    DPIF_RADARDEMO_DETECTIONCFAR2_TOP = 0xFFFFFFFF
} DPIF_RADARDEMO_detectionCFAR_Type;

/**
 *  \enum
 *   {
 *  RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP = 0,
 *  RADARDEMO_DETECTIONCFAR_INPUTTYPE_NOT_SUPPORTED
 *   }   RADARDEMO_detectionCFAR_inputType;
 *
 *  \brief   enum for CFAR input type.
 *
 *
 */

typedef enum
{
    DPIF_RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP = 0, /**< input type: single precision floating point*/
    DPIF_RADARDEMO_DETECTIONCFAR_INPUTTYPE_NOT_SUPPORTED,
    DPIF_RADARDEMO_DETECTIONCFAR_INPUTTYPE_TOP = 0xFFFFFFFF
} DPIF_RADARDEMO_detectionCFAR_inputType;


    typedef struct CLI_RADARDEMO_aoaEst2D_rangeAngleCfg_t
    {
        float   searchStep; /**angle search resolution */
        float   mvdr_alpha; /**diagonol loading weight.*/
        uint8_t detectionMethod; /**< detection method,
                                                               0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
                                                               1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
                                                               2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation. */
        uint8_t dopplerEstMethod; /**< Doppler estimation method, 0-single peak search, 1-CFAR.*/
    } CLI_RADARDEMO_aoaEst2D_rangeAngleCfg;


    typedef struct CLI_RADARDEMO_detectionCFAR_config_t
    {
        uint32_t                          fft1DSize; /**< 1D FFT size*/
        uint32_t                          fft2DSize; /**< 2D FFT size*/
        uint32_t                    cfarType; /**< Type of CFAR.*/
        uint32_t                    inputType; /**< Type of integration.*/
        float                             pfa; /**< Desired false detection ratio.*/
        float                             K0; /**< Relative detection threshold. If K0 is non-zero value, pfa setting will be ignored.*/
        float                             rangeRes; /**< Range resolution.*/
        float                             dopplerRes; /**< Doppler resolution.*/
        float                             dopplerSearchRelThr; /**< Doppler search relative threshold.*/
        uint8_t                           enableSecondPassSearch; /**< Flag for enabling second pass search, if set to 1. If set to 0, no second pass search*/
        uint8_t                           searchWinSizeRange; /**< Search window size for range domain search.*/
        uint8_t                           guardSizeRange; /**< Number of guard samples for range domain search.*/
        uint8_t                           searchWinSizeDoppler; /**< Search window size for Doppler domain search.*/
        uint8_t                           guardSizeDoppler; /**< Number of guard samples for Doppler domain search.*/
        uint8_t                           searchWinSizeNear; /**< Search window size for near range domain search.*/
        uint8_t                           guardSizeNear; /**< Number of guard samples for near range domain search.*/
        uint16_t                          maxNumDetObj; /**< maximum number of detected obj.*/
        uint8_t                           leftSkipSize; /**< number of samples to be skipped on the left side in range domain. */
        uint8_t                           rightSkipSize; /**< number of samples to be skipped on the right side in range domain. */
        uint8_t                           leftSkipSizeAzimuth; /**< number of samples to be skipped on the left side in azimuth domain. */
        uint8_t                           rightSkipSizeAzimuth; /**< number of samples to be skipped on the right side in azimuth domain. */
        uint32_t                          log2MagFlag; /**<use log2(mag) as input*/
        uint32_t                          shortened1DInput; /**<Flag if set to 1, to indicate that the heatmap is already trimmed by skip left sample at 1D dimmension*/
        uint32_t                          angleDim1; /*Dim1 of angle, for removing non-local max side peaks in angle domain*/
        uint32_t                          angleDim2; /*Dim2 of angle, for removing non-local max side peaks in angle domain*/
        uint8_t                           rangeRefIndex; /* Index of the range bin to be used as a reference point when processing other range bins*/
    } CLI_RADARDEMO_detectionCFAR_config;

    typedef struct CLI_RADARDEMO_aoaEst2D_2DAngleCfg_t
    {
        float   elevSearchStep; /**eleveation search resolution */
        float   mvdr_alpha; /**diagonol loading weight.*/
        uint8_t maxNpeak2Search; /**< Max number of peak to search, max at 6. */
        uint8_t peakExpSamples; /**< neighbor ppoint to do peak expansion on each side.*/
        uint8_t elevOnly; /**< elevation estimation only */
        float   sideLobThr; /**Sidelobe threshold */
        float   peakExpRelThr; /**peak expansion relative threshold -- only include neighbors with power higher than  peakExpRelThr * peakPower*/
        float   peakExpSNRThr; /**peak expansion SNR threshold -- only expand peak with SNR higher than this threshold */
    } CLI_RADARDEMO_aoaEst2D_2DAngleCfg;

    typedef struct CLI_RADARDEMO_aoaEst2D_2DZoomInCfg_t
    {
        uint8_t zoominFactor; /**< Zoom in factor */
        uint8_t zoominNn8bors; /**< number of neighbors to zoom in on each side.*/
        uint8_t peakExpSamples; /**< neighbor ppoint to do peak expansion on each side.*/
        float   peakExpRelThr; /**peak expansion relative threshold -- only include neighbors with power higher than  peakExpRelThr * peakPower*/
        float   peakExpSNRThr; /**peak expansion SNR threshold -- only expand peak with SNR higher than this threshold */
        uint8_t localMaxCheckFlag; /**Loca max check flag: 0 - no check; 1 - elevation domain only; 2 - both elevation and azimuth */
    } CLI_RADARDEMO_aoaEst2D_2DZoomInCfg;

    //! \brief   Configuration substructure RADARDEMO_aoaEst2D_staticCfg for RADARDEMO_aoaEst2DCaponBF configuration.
    //!
    typedef struct CLI_RADARDEMO_aoaEst2D_staticCfg_t
    {
        uint8_t staticProcEnabled; /**< Enable static scene processing if set to 1 */
        uint8_t staticAzimStepDeciFactor; /**< static azimuth search step decimation factor, over the azimuth search steps in RADARDEMO_aoaEst2D_rangeAngleCfg*/
        uint8_t staticElevStepDeciFactor; /**< static elevation search step decimation factor, over the elevation search steps in RADARDEMO_aoaEst2D_2DAngleCfg*/
    } CLI_RADARDEMO_aoaEst2D_staticCfg;

    //! \brief   Configuration substructure RADARDEMO_aoaEst2D_staticCfg for RADARDEMO_aoaEst2DCaponBF configuration.
    //!
    typedef struct CLI_RADARDEMO_aoaEst2D_dopCfarCfg_t
    {
        uint16_t cfarDiscardLeft; /**< Number of left cells discarded.*/
        uint16_t cfarDiscardRight; /**< Number of left cells discarded.*/
        uint16_t refWinSize; /**< reference window size in each side in two directions for clutter variance estimation. */
        uint16_t guardWinSize; /**< guard window size in each side in two directions. */
        float    thre; /**< threshold used for compare. */
    } CLI_RADARDEMO_aoaEst2D_dopCfarCfg;

    typedef struct CLI_RADARDEMO_doppBinSel_config_t
    {
        uint16_t doppBinSelEnable; /**< Number of input range bins.*/
        uint16_t doppFFTSize; /**< Doppler FFT size (zero padding).*/
        uint16_t doppSelMinBin; /**< Doppler Bin min bin select (positive).*/
        uint16_t doppSelMaxBin; /**< Doppler bin max bin select (positive).*/
    } CLI_RADARDEMO_doppBinSel_config;

    /*
     * @brief Main DSP Configuration structure
     */
    typedef struct DPIF_MSS_DSS_PreStartCfg_t
    {
        // rangeFFT/Doppler parameters
        float    framePeriod; /**< Frame period in msec. */
        uint16_t numAdcSamplePerChirp; /**< number of adc samples per chirp. */
        uint16_t numAdcBitsPerSample; /**< number of adc bits per sample. */
        uint16_t numChirpPerFrame; /**< number of chirps per frame. */
        uint16_t numTxAntenna; /**< number of antennas. */
        uint16_t numAntenna; /**< number of virtual antennas. */
        uint16_t numPhyRxAntenna; /**< number of physical RX antennas. */
        uint16_t mimoModeFlag; /**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
        uint32_t numTotalChirpProfile; /**<number of chirp profiles*/
        uint32_t numUniqueChirpProfile; /**<number of unique chirp profiles*/
        uint16_t numFrmPerSlidingWindow;
        float    chirpInterval;
        float    bandwidth;
        float    centerFreq;

        CLI_RADARDEMO_detectionCFAR_config dynamicCfarConfig;
        CLI_RADARDEMO_detectionCFAR_config staticCfarConfig;

        float dynamicSideLobeThr; /**< CFAR sidelobe threshold for dynamic scene. */
        float staticSideLobeThr; /**< CFAR sidelobe threshold for static scene. */

        /* DOA Config */
        CLI_RADARDEMO_aoaEst2D_rangeAngleCfg rangeAngleCfg;
        union
        {
            CLI_RADARDEMO_aoaEst2D_2DAngleCfg  azimElevAngleEstCfg; /**< azimuth elevation angle estimation config.*/
            CLI_RADARDEMO_aoaEst2D_2DZoomInCfg azimElevZoominCfg; /**< azimuth elevation angle zoom in config.*/
        } angle2DEst;

        CLI_RADARDEMO_aoaEst2D_staticCfg  staticEstCfg; /**< static scene angle estimation config.*/
        CLI_RADARDEMO_aoaEst2D_dopCfarCfg dopCfarCfg; /**< configurations, if CFAR is selected to use for Doppler estimation.*/
        CLI_RADARDEMO_doppBinSel_config   doppBiningCfg; /**< Doppler binning configuration for Capon. */

        float phaseCompVect[2*SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL]; /**< antenna partern configuration: phase compensation vector -- board dependant. (Imaginary first then Real)*/

        int8_t  m_ind[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL]; /**< antenna partern configuration 1 .*/
        int8_t  n_ind[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL]; /**< antenna partern configuration 2.*/
        int8_t  phaseRot[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL]; /**< antenna partern configuration: phase rotation.*/
        float   fovCfg[2]; /**< antenna partern configuration: field of view for azimuth fovCfg[0], and elevation fovCfg[1].*/


        uint16_t numRangeBins; /**< number of range bins, output from the init function -- in case to be used in framework. */
        uint16_t rangeFftSize; /**< range FFT size. for complex samples numTangeBins = rangeFftSize */
        uint16_t maxNumDetObj; /**< max number of detected points. */

        /*! @brief   Radar cube */
        DPIF_RadarCube radarCube;

        uint8_t exportCoarseHeatmap;
        uint8_t exportRawCfarDetList;
        uint8_t exportZoomInHeatmap;

        /*! For debugging only, normally set to zero */
        uint8_t disablePointCloudGeneration;
    }  DPIF_MSS_DSS_PreStartCfg;


    // detection heatmap 3D
    typedef struct DPIF_MSS_DSS_detHeatmap3D_t
    {
        uint32_t    numRangeBins;
        uint32_t    numAzimuthBins;
        uint32_t    numElevationBins;
        float       *data;
    } DPIF_MSS_DSS_detHeatmap3D;

    // Raw CFAR detection list structure
    typedef struct DPIF_MSS_DSS_rawCfarDetPoint_t
    {
        uint16_t    rangeInd;
        uint16_t    angleInd;
        float       snr;
    } DPIF_MSS_DSS_rawCfarDetPoint;

    // Raw CFAR detection point-cloud list structure
    typedef struct DPIF_MSS_DSS_rawCfarPointCloud_t
    {
        int32_t                  object_count; // number of objects (points)
        DPIF_MSS_DSS_rawCfarDetPoint *list;
    } DPIF_MSS_DSS_rawCfarPointCloud;


    // Output point cloud structure
    typedef struct DPIF_MSS_DSS_pointCloud_t
    {
        int32_t                  object_count; // number of objects (points)
        DPIF_PointCloudSpherical pointCloud[DPIF_DOA_OUTPUT_MAXPOINTS];
        DPIF_PointCloudSideInfo  snr[DPIF_DOA_OUTPUT_MAXPOINTS];
    } DPIF_MSS_DSS_pointCloud;

    typedef struct DPIF_MSS_DSS_radarProcessBenchmarkElem_t
    {
        uint32_t dynNumDetPnts;
        uint32_t dynHeatmpGenCycles;
        uint32_t dynCfarDetectionCycles;
        uint32_t dynAngleDopEstCycles;
        uint32_t staticNumDetPnts;
        uint32_t staticHeatmpGenCycles;
        uint32_t staticCfarDetectionCycles;
        uint32_t staticAngleEstCycles;
    } DPIF_MSS_DSS_radarProcessBenchmarkElem;

    typedef struct DPIF_MSS_DSS_radarProcessOutput_t
    {
        DPIF_MSS_DSS_pointCloud                 pointCloudOut;
        DPIF_MSS_DSS_radarProcessBenchmarkElem  *benchmarkOut;
        DPIF_MSS_DSS_detHeatmap3D               heatMapOut;
        DPIF_MSS_DSS_rawCfarPointCloud          rawCfarPointCloud;
    } DPIF_MSS_DSS_radarProcessOutput;
/**
@}
*/
#ifdef __cplusplus
}
#endif

#endif /* DPC_OBJECTDETECTION_H */
