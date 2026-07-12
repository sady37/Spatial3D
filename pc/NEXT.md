# NEXT — 雷达生命体征后续任务

接续:实时连续 HR 输出 + 心动过速。本文件是新会话的提示词/交接。

## ✅ 已完成(2026-07-11 合并 web 分支)

- **AF 房颤判别**(从 web `hr_continuous.py` 移植进 `bcg_vitals_rt.py`,纯附加不碰核心):
  两级判决 — ① **存在性门**(尺度无关:心脏带/呼吸带能量比 ≥0.90 才判,只问"有没有
  心脏信号"、不惩罚不规律);② **规整度分类**(SQI 加权融合谱集中度+熵)。滚动 7 窗 ≥4
  才报 AF ALERT。**四份真实静态全程 `indeterminate`(正确弃权)、0 持续警报** —— 3.5m
  座姿心脏 SNR 太低判不了节律,这是安全行为不是失败。阈值(0.90/0.60/0.45)是合成+单份
  真实标定的**脚手架**,需真实房颤(或近距离强窦性)采集锚定。`af_metrics()` 每窗输出。
- **峰插值 refinement**:核心 `autocorr_peak(..., interp=True)`(默认 False → main 逐字节
  不变;RT 传 True)——对自相关峰做抛物线亚 lag 插值,消掉 80bpm 附近 ~6bpm 整数-lag 台阶。
  仅细化静息带连续曲线平滑度;tachy 值仍走 FFT。
- **回归确认合并无破坏**:核心 main 81/81/87/81 不变;RT 中位 sit39 78.4/sidesit 78.2/
  lie41 76.8/fall20 80.1(对齐验证)。**已知**:RT 里 lie41/fall20 有 5/14 个瞬时 HIGH
  窗(原提交即有,非本次引入),被连续性+卡尔曼吸收 → 最终 HR 仍静息;若将来把逐窗 band=HIGH
  当事件上报,需先加去抖(连续 N 窗才算 tachy 事件)。

## ✅ 已完成(2026-07-11 本轮)

- **核心重构(零行为变化)**:`bcg_vitals.py` 把相位解调/RR/HR 抽成可复用函数
  `demod_channels / estimate_rr / hr_band_search / estimate_hr`,main 输出逐字节不变
  (四份回归 81/81/87/80 精确不动)。RT 层 import 这些函数,**未碰已验证核心**。
- **任务 1 = `bcg_vitals_rt.py`**(滑窗连续 HR + 时序层)。15s 窗/1.5s 步,每窗跑
  `estimate_hr` → 瞬时 HR+bin-spread+autocorr 峰高(SQI 代理)。时序层三件套:
  ① **卡尔曼**(标量随机游走,测量噪声 R 随 bin-spread 增大 → 低置信窗少动状态);
  ② **连续性验证**(近 5 值,±10% 均值 / ±20% 上值,不过则不输出、卡尔曼滑行);
  ③ **质量门 + 备份**:autocorr 峰高 < q_min(默认 0.40)= 掉线检测(**关键发现:HR 值本身
     被 band-prior 钳在 60-102bpm,纯噪声也给"貌似合理"的值 → 值/spread 都测不出掉线,
     只有 autocorr 峰高/带内 SNR 能测**);连续失败 ≥3 → 更长窗(30s)重捕获重锚。
  用法 `python bcg_vitals_rt.py sit39_cube.npz --fps 18.8 --plot hr.png`。
  四份连续曲线:sit39 中位 79、sidesit 78、lie41 77、fall20 80(全 ≈ 验证值);
  合成掉线测试(注噪 12s)证明 lowconf/coast + 备份重捕获 + 卡尔曼稳态保持。
  图:`hr_timeline_{sit39,sidesit,lie41,fall20,disturbed}.png`。
- **任务 2 算法 = `estimate_hr(..., tachy_hi=2.2)` 高带仲裁**(`--tachy 2.2` 开关,两处都通)。
  做法:低边**恒定 1.0Hz**(0.7-1.0 呼吸残留永不重入),仅当 SQI-top bins 里
  **宽带自相关周期的中位 > 1.7Hz 且多数(≥50%)bins 落在 1.7Hz 之上**才判心动过速(区域投票);
  判定后 tachy 值用**该带 FFT 峰**(自相关在 102-132bpm 只有 ~3 个整数 lag,太粗;FFT 细)。
  **走过的死路**:自相关"峰高比"和窄带"谱突出度(PMR)"都**不能**判高/低带 —— 二者对
  band-limited 残留都饱和(高带 PMR≈低带,峰高甚至更高 → 假心动过速 113);**只有宽带周期落点**能判。
  四份回归全判 LOW、HR 不变(sidesit 38%、lie41 38%、fall20 50%+中位98<102 均被挡)。
  **仍缺**:真高心率数据验证 tachy 的**数值**(见下)。

### 遗留(任务 2 收尾,需硬件)
运动后立即录一份 cube(`python cap_cube.py tachy1_cube.npz 120`),同步 Apple Watch,
跑 `bcg_vitals.py tachy1_cube.npz --fps <实测> --tachy 2.2` 看是否判 HIGH 且 FFT 值对表。
若边界误判:调 `estimate_hr` 的 `vote_frac`(默认 0.5)或 tachy_hi。RT 连续版同 `--tachy 2.2`。

---
（以下为本轮开始前的原始交接,保留）

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
