# NEXT — 雷达生命体征后续任务

接续:实时连续 HR 输出 + 心动过速。本文件是新会话的提示词/交接。

## 现状(已完成、已验证)

- **HR/RR 管线** `pc/bcg_vitals.py`。配方 = **相位解调(mm 通道)→ SQI 选胸腔 bin → 自相关@生理带 [1.0-1.7Hz] 取中位**。RR 用低频带中位数(非 SQI 加权)。
- **已对 Apple Watch 三姿势验证**(单次自动跑,全 match):
  | 姿势 | 雷达 HR | Apple Watch |
  |---|---|---|
  | 正坐 3.9m | 81 | 80-85 |
  | 半侧身 3.9m | 81 | 83-91 |
  | 躺 4.1m | 84 | 79-83 |
- **采集** `pc/cap_cube.py <out.npz> <秒> [--cfg <20fps.cfg>]` —— 存完整 slow-time cube(`snapshots`+`covariances`+`mean`)。20fps/44-bin(3.5-4.5m)profile = `profile_fall_20fps_gaze.cfg`。切 fps/窗**必须断电重启**后首份下发(stock demo 流式中途不服务 CLI)。
- **测试数据**(pc/):`sit39_cube.npz`(正坐 240s@18.8fps)、`sidesit_cube.npz`(半侧身 120s)、`lie41_cube.npz`(躺 180s)、`fall20_cube.npz`(躺+拉动 120s)。
- **参考算法**:`../sleep算法.md` + `../sleep_pad_algorithm_implementation.md`(床垫 BCG:三重估计 + SQI + 卡尔曼 + 连续性验证 + 备份)。
- **已排除的死路(别重追)**:FFT argmax(减半)、harmonic-sum(要手调范围才对)、coprime/CRT 折叠(数学对但解的是最强干扰的八度,治不了 SNR)。**唯一 lever 是生理带先验**;根因是 4m 处心跳相位弱于呼吸谐波残留。详见 memory `vital-signs-mode-design`。
- **几何**:HR 抗角度(正/侧/躺都出);RR 侧身退化(胸腔切向,自动标 LOW)。坐姿 ≫ 躺姿 SNR。

## 任务 1:实时连续 HR 输出

把"整段一个 HR"改成**滑窗连续**:
1. 滑窗(15s 窗、1-2s 步长),每窗跑 `autocorr@[1.0-1.7Hz]` 出瞬时 HR + bin-spread 置信。
2. 接 `sleep算法.md` 后处理:
   - **卡尔曼平滑**(状态=HR,过程噪声小、测量噪声按置信度调,抑制跳变);
   - **连续性验证**(当前 vs 近 5 历史,±10%/±20% 门限,离群标"可疑"不立即输出);
   - **备份机制**(连续验证连续失败 → 回退到更长窗/谱估计的趋势值,处理翻身后重捕获)。
3. 用 `sit39_cube.npz`(4min)当连续输入,画 **HR-vs-time 曲线**验证平滑+跟随。

## 任务 2:心动过速(放宽上限)

现 `HR_PHYS_HI=1.7Hz`(102bpm)会截断快心率。放宽(如 2.2Hz/132bpm)时**低边仍要挡住 0.7-1.0Hz 呼吸谐波残留**,别让搜索带重新掉坑。做法二选一:
- **动态低边**:先估 RR→按 ~5·f0 定低边(避开呼吸密集谐波区);
- **高带优先逻辑**:若高带(>1.7Hz)有比低带更强、bin 间更一致的自相关峰,则采信高带。

造/采一份**高心率数据**(运动后立即录)对 Apple Watch 验证。

## 约束

**别改已验证的相位解调 + band-prior 核心**,只在其上加**时序层**(卡尔曼/连续性)和**上限自适应**。改动后必须四份数据回归(都应 ~81)+ 有真值的对 Apple Watch。
