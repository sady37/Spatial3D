# people_tracking_6844 — Spatial3D 固件 version（权威基准）

自包含、完整的固件**源码 version**：我们在 TI `IWRL6844_People_Tracking` demo 上打的 Spatial3D
补丁的**唯一权威基准**。`ti_ref/` 整棵是第三方参考树、被 `.gitignore` 忽略；我们改的固件源码曾
经因此不在 git 里，导致没有基准、只能从 VM 备份反推。这个目录解决它。

## 同步纪律（重要）
- **VM 是构建/验证的地方**（Ubuntu VM，见 memory `vm-access-map` / `firmware-build-workflow`）。
- **每次 VM 上的所有变更，验证通过后，手动 cover 回这个 version**，然后 commit。
- 反过来:要改固件，就改这里(或 `ti_ref` 工作副本),scp 到 VM 构建;通过后再落回这个 version。
- 目标恒等式:**git(本 version) == VM 源码 == 已烧录固件的源码**。不允许再出现三者漂移。
- 只放**源码部分**(src 树 + chirp_configs)。docs / prebuilt_binaries / SDK 不进 version(第三方,
  留在 `ti_ref` 或 SDK 安装里)。

## 目录内容
- `src/6844/` — 完整可编译源码:`mss/`(R5F)、`dss/`(C66x)、`common_mss_dss/`、三个 `*.projectspec`、
  `mss/.../ti-arm-clang/linker.cmd`。这是 cp 到/从 VM 的那份。
- `chirp_configs/` — 驱动 cfg(`sbr_3dpt_5m.cfg` = 近场/vitals/fall/per-bin-cube；`_10m` = 大范围)。
  注:出厂 cfg 不含 `cubeQuery`(那是运行时命令,由 server 触发,见下)。

## 当前 version 状态 — Phase 1（cubeQuery）
**已构建 + 已 stage,尚未硬件 flash 验证**(2026-07-16)。烧录后跑验收:见 memory `track-bin-cube-patch`。
- Appimage: VM `/media/sf_share/people_tracking_6844_CUBEQUERY.release.appimage`
  (md5 `7cc46b01ab0fd5dcdd1d9d47ae6d7d7a`),用户从 Win10 flash。
- 验收: 重启 `pc/web/radar_server.py live` 后
  `curl "http://127.0.0.1:8765/api/cube?bin=36&n=30&hw=3"` → `entries>0`(躺地/无 track 也能抽)。

### 这个 version 相对 TI 原始 demo 的补丁
1. **cubeQuery（Phase 1 核心，track-independent）** — 新 CLI `cubeQuery <range_bin> <half_win>
   <n_frames>`(`MmwDemo_CLICubeQuery`);MCB `tbcQuery{Active,Bin,HalfWin,FramesLeft}`;
   `DPC_Execute` 每帧武装时用 `MmwDemo_tbcExtractBin` 从 `radarCube[0]` 抽 bin±halfWin(tid=0)、
   连发 N 帧 TLV 320 后自停。**认 range 数字,不认 track**(摔倒 track 冻结也不受影响)。
   TLV 320 emit 门改为 `tbcNumEntries>0`。删除了原来的 fall 状态机(velZ/height ARM/CONFIRM/BURST,
   数据证伪)。`trackBinCubeCfg` 保留为 no-op stub(旧 cfg 兼容)。
2. **FIX2 栈搬移** — `gDspPointCloudTaskStack` 加 `section(".bss.dsp_tcmb")`(mmwave_demo_mss.c)
   + linker.cmd 规则 `.bss.dsp_tcmb {} align(32) > TCMB_RAM`,把 28KB 分类器栈从 TCMA 挪到 TCMB。
   删掉 fall SM 后 TCMA 空闲 5 B→**5854 B**(map 实测),启动安全。见 memory `fallsm-boot-bug`。
3. **SDK-skew 修复(6.0.5.1→VM 的 6.0.4.1)** — `mmwave_demo_mss.c:280` `MMWave_stop` 3→2 参;
   `projectspec` 补 `common_test.c` 的 `<file>` 项;`common_test.c` 去掉重复的 app_ioWrite/ioRead、
   只留 unityCharPut + critSec stubs。这些原本只在 VM 上、没回 Mac —— 已收进本 version。

## 构建 recipe（VM，CCS headless）
见 memory `track-bin-cube-patch`(BUILD RECIPE 段)。要点:CCS toolbox 的 makefile 是坏模板,别用;
在 ccs_ws mss `Release/` 里 `gmake -k -j4 all` 构建 mss rig;system-post-build 的 metaImage 步骤坏
(JSON 被 echo 弄乱),要**手动** `metaImage_creator --complete_metaimage <abs-cfg>` 合成 appimage。
DSS 未改时复用旧 dss rig。
