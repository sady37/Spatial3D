# mmw_demo firmware mod — range-antenna zero-Doppler TLV (type 8)

Adds the **single firmware modification** for the Spatial3D three-mode architecture
(see `AWRL6844.md` §5.5): a new output TLV carrying the zero-Doppler (static) 16-virtual-
antenna complex vector for a configurable window of range bins, so the server can run
MUSIC DOA offline.

Source of truth is the TI SDK on the Linux VM:
`~/ti/mmwave_l_sdk_06_00_04_01/examples/mmw_demo/mmwave_demo/source/`
`orig/` = pristine pulls, `mod/` = edited, `*.patch` = unified diffs. Originals are also
backed up on the VM as `*.spatial3d.bak`.

## What changed (3 files)

- **mmwave_demo.h** — `CLI_GuiMonSel` gains `rangeAntennaStartBin` / `rangeAntennaNumBins`
  (uint16). The `rangeAzimuthHeatMap` byte is reused as the enable gate.
- **mmwave_demo.c** — `MmwDemo_transmitProcessedOutputTask()` emits TLV type 8
  (`MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP`, value 8).
- **mmw_cli.c** — new CLI command `rangeAntennaOutput <startBin> <numBins> <enable>`.

## TLV type 8 wire format (little-endian)

    [uint16 start_bin][uint16 num_bins]
    then, per bin: 16 × cmplx16ImRe_t  (int16 imag, int16 real)   -- imag FIRST

Each 16-vector is the **coherent mean over the chirp dimension** of the 1D range-FFT
radar cube (`DPIF_RADARCUBE_FORMAT_2`: `x[numRangeBins][numDopplerChirps][numTx][numRx]`),
i.e. the zero-Doppler / static component. Virtual antenna index = `tx*numRx + rx`.
int16 is lossless (ADC is 12-bit); float32 would waste 2× UART bandwidth.

Parser + contract test: `pc/spatial3d/tlv.py` (`parse_range_antenna`, `Frame.range_antenna`)
and `pc/tests/test_tlv.py::test_range_antenna_roundtrip`.

## Bandwidth note

At 1.25 Mbaud (~125 KB/s) the full range profile (all bins × 16 ant × 4 B) exceeds the
10 fps frame budget — hence the `numBins` window. The demo already warns
("Frame Time is not enough...") if the configured TLVs overflow the frame period.

## Build (blocked — toolchain not yet installed on the VM)

The Ubuntu VM has the SDK source + `make` but **no compiler/sysconfig**:
missing `ti-cgt-armllvm_4.0.2.LTS`, `sysconfig_1.23.0` (needs bundled node), no CCS.
Install those (paths per `~/ti/mmwave_l_sdk_06_00_04_01/imports.mak`) then:

    cd ~/ti/mmwave_l_sdk_06_00_04_01/examples/mmw_demo/mmwave_demo/xwrL684x-evm/r5fss0-0_freertos/ti-arm-clang
    make    # -> mmwave_demo.release.appimage

Flash the appimage from the Win10 VM (TI visualizer / UniFlash) as before.

## CLI usage (after flashing)

Add to the `.cfg` before `sensorStart`, e.g. export 100 bins starting at bin 0:

    rangeAntennaOutput 0 100 1

`0 0 0` (or omitting the line) disables the TLV → point-cloud-only modes.
