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

## 当前 version 状态 — Phase 2（per-track pose MLP）
**已实现 + host 验证,尚未 VM 编译/硬件 flash**（源码已在本 version）。在 6844 People_Tracking
上加一条**每 track 姿态分类**辅助腿:4 类 MLP(Stood/Sat/Lying/Falling)在 R5F MSS 上原生跑,
结果按 tid 挂到新 TLV 321。定位:**白得 pose + 摔倒动作触发**的辅助腿,主 fall 判据仍在 server
(window + cube-RR)。见 memory `fall-detection-design` / `fall-modular-pipeline`。

### 为什么是重训 + 自写 forward,而不是移植 TI 的 pose_model.a
TI `Pose_And_Fall` 的 `pose_model.a` 是 **Cortex-M4 / v7E-M Thumb-only** 目标(解析其 ARM build
attributes 确认:`Tag_CPU_arch=13 (v7E-M)`, `Tag_CPU_arch_profile='M'`),**链不进 6844 的 R5F /
ARMv7-R 镜像**。TI 也没随附 `.pth`/`.onnx`,只有 `.a` 里的 TVM rodata + classes.zip 数据集。
所以:用 classes.zip 重训 → 自己导权重 → 自己写 forward,`.a` 和 TVM runtime 全不碰。

### 训练/导出(在 Mac,`pc/pose/`)
- `pc/pose/dataset.py` — TI 特征提取的清洗版。**三处清洗**(每处对应 classes.zip 里实测的缺陷):
  (1) 丢掉「posz 轴死掉」的录制(整文件 |posz|<10cm;按类干净切分 → standing/sitting/lying 0%,
  falling 30.5%,walking 51.2% → 是**标签捷径**,6844 上 posz 恒活会漏);(2) 丢掉 vel/acc 到 2.4e35
  的毒行(TI 的 FILTER 只管 posz,不管 vel/acc → 3 行进 BatchNorm 会 var=inf);(3) 帧窗只在**同一
  文件内连续帧**上开。**特征从 22 降到 20**:去掉 velx/accx —— 它们只有 `replay_*` schema 记录,
  `results_*` 缺列被 TI 填 0,导致「精确 0.0」在 non-falling 占 98%、falling 仅 77%(AUC 0.76 的
  schema 泄漏),而 std(non-Fall)=2.7e-5 让 BatchNorm 放大 ~95×。**砍掉 walking**(清洗后只剩 ~2
  session,必过拟合)。
- `pc/pose/model.py` — TI 架构(bn1 输入归一化 → 160→64→32→16→4),但 forward 返回 logits(TI 原版
  softmax+CrossEntropy 双 softmax,梯度被压);BN 在导出时**折叠进 Linear**(`fold_bn`)。
- `pc/pose/train.py` — 训练 + 双口径报准确率:**random-row(TI 的方法,虚高)vs grouped-by-file
  (诚实)**。实测 **97.8–98.2% grouped**(vs 99.9% random-row);Falling 召回 97.6%、Lying 100%
  (清洗后 Lying 从 TI 的 63% 修复,因为高度轴不再被 schema 污染)。⚠️ grouped-by-**file** 仍共享
  TI 那 5 个人(train/test 同人),所以是乐观上界,但远比 99.9% 诚实。
- 导出:`python -m pose.train --data <classes解压目录> --out .../pose/pose_model.c`。
  BN-fold 与 torch 逐点对齐 <3e-7(train.py 内断言)。
- **CAVEAT**:权重来自 TI 6432(8 天线、TI mount)。已清洗,但仍是 6432 几何 → 6844 上必须用
  `poseCfg` 的 zOffset 标定,让站立读到 posz ≈ +0.33m。
- **重训环境**:Intel Mac 上 torch 停在 2.2.2(需 numpy<2),与主 venv(numpy 2.1)冲突 → 用独立
  venv `pc/.venv-pose`(gitignored)。主 venv(`pc/.venv`)跑 server/tlv 测试。

### 固件侧改动(4 处 + 新目录 `mss/source/pose/`)
1. **pose/pose_mlp.{c,h} + pose/pose_model.c**(生成)— 折叠后 forward(纯 乘加+relu+softmax)+
   **每 track 20×8 环形缓冲**(`POSE_MAX_TRACKS=8`,~6.5KB .bss)。每 track:把点云按半径 0.75m 门到
   该 track,取最高 5 点建 20 特征帧,压入环形缓冲;满 8 帧才推理。TI 原版是单 track(`tList[0]`),
   这里泛化到多 track。权重放 `.rodata.pose_model`(50.7KB)。**零拷贝**:点云用 `PosePointGet`
   访问器**就地读** `dpcAoAObjOutCartExt`,不建 PosePoint 拷贝数组(否则最坏 2000 点×16B=32KB scratch)。
   core 不 #include SDK 类型 → host 可编/可测。
2. **linker.cmd** — 新规则 `.rodata.pose_model: {} palign(8) > TCMB_RAM`,放在通用 `.rodata`
   GROUP **之前**(first-match),把 51KB 权重**钉在 TCMB**。绝不能溢到 TCMA(cubeQuery 后只剩
   ~5.8KB,溢 51KB 直接 brick,见 `fallsm-boot-bug`)。`.bss.pose`(~6.5KB 环形+scratch+poseKin)同钉 TCMB。
3. **dpc_mss.c** — `DPU_TrackerProc_process` 之后,由 `tList`(trackerProc_Target)建 per-track
   kinematics(仅 256B 小拷贝),点云传 `dpcAoAObjOutCartExt` 指针 + `MmwDemo_poseGetPoint` 访问器
   (就地读 + snr ×0.1 把 0.1dB-steps 换成 classes.zip 的 dB 尺度),调 `PoseMlp_process`,结果存 MCB。
4. **mmwave_demo_mss.{c,h}** — TLV 321 enum + MCB 字段(`poseEnable`/`poseNumResults`/
   `poseResults[]`)+ header/write 两趟 emit;`MMWDEMO_OUTPUT_ALL_MSG_MAX` 11→14。
5. **mmw_cli.c** — `poseCfg <enable> [zOffset_cm]`(`MmwDemo_CLIPoseCfg`):重置环形缓冲 +
   设 zOffset(cm→m)+ 开关。发在 sensorStart 前。

### TLV 321 契约（little-endian,给 server `pc/spatial3d/tlv.py`）
```
uint16 numResults;    // 本帧有 pose 的 track 数(≤ POSE_MAX_TRACKS=8)
uint16 reserved;      // 0,保持后面数组 4 字节对齐
然后 numResults × PoseResult(每个 8 字节):
    uint32 tid;          // track id(对应 TLV 308 的 tid)
    uint8  pose;         // 0=Stood 1=Sat 2=Lying 3=Falling, 0xFF=unknown(窗未满)
    uint8  fallingProb;  // P(Falling)×255,0..255
    uint8  valid;        // 1=本帧真推理,0=窗未满/点不足
    uint8  pad;          // 0
```
每 entry 8 字节。`valid=0` 时 pose=0xFF。server:`Frame.poses()` → `{tid: Pose}`;`/api/scene` 每
track 带 `pose`(标签串)+ `falling_prob`。录制 npz 新增 `t_pose`/`t_fprob` 列(对齐 `t_*` 轨迹列)。

### 验收(硬件)
flash 后:`poseCfg 1 30`(30cm 是占位 zOffset,需按 mount 标定)→ 重启 `radar_server.py live` →
`/api/scene` 每 track 出现 `pose` 字段;静坐读 Sat、站立 Stood、躺地 Lying/Falling。
host 侧已验证(build 前就证明逻辑对):C `poseInfer` vs Python folded_forward 逐点 1e-6;整条
`PoseMlp_process`(门控+环形+flatten+推理+falling量化)vs Python 参考 end-to-end MATCH
(`pc/pose/` + scratchpad host harness)。

### VM 构建注意（Phase 2 特有）
- 新增 `mss/source/pose/{pose_mlp.c,pose_mlp.h,pose_model.c}` 已加进 mss projectspec 的 `<file>`
  项(`targetDirectory="pose"`)。**projectImport 会把 .c 拷进 ccs_ws** → 记得把这三个新文件 +
  改过的 `dpc_mss.c`/`mmwave_demo_mss.c`/`mmw_cli.c` scp 进 ws build 副本(见 `track-bin-cube-patch`
  的 build-workspace note:改 toolbox src 不自动进 build)。`mmwave_demo_mss.h`/`pose_mlp.h` 走 `-I`
  从 toolbox src,头改动能进 build。
- **VM 编译要核对的一件事**:`trackerProc_Target` 的字段名。本 version 用 `tid/posX/posY/posZ/
  velY/velZ/accY/accZ`(vital_signs guide 的 targetStruct3D + TI 6432 pose demo 注释均如此)。Mac 上
  没有 SDK 头无法预检;若 VM 报字段名错,是一行改名的事(如 `tid`→`trackerID`)。
- 内存:pose 共用 TCMB ~57KB(rodata 51KB + `.bss.pose` ~6.5KB),256KB 富余。核对 map 里 TCMA
  free 仍 ~5.8KB —— pose 的 rodata + .bss + dpc 里的 `poseKin` 都已显式钉 `.bss.pose`/
  `.rodata.pose_model` → TCMB,**不应有任何 pose 段进 TCMA**;若 map 显示有,查 linker first-match。

## 构建 recipe（VM，CCS headless）
见 memory `track-bin-cube-patch`(BUILD RECIPE 段)。要点:CCS toolbox 的 makefile 是坏模板,别用;
在 ccs_ws mss `Release/` 里 `gmake -k -j4 all` 构建 mss rig;system-post-build 的 metaImage 步骤坏
(JSON 被 echo 弄乱),要**手动** `metaImage_creator --complete_metaimage <abs-cfg>` 合成 appimage。
DSS 未改时复用旧 dss rig。
