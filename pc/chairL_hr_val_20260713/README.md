# chairL 静息 HR 验证 — 2026-07-13 夜

雷达盲态静息 HR（选胸部 bin + autocorr）对 **Apple Watch** 逐窗验证。全程盲态：选 bin、估 HR 都不看真值，真值只用于打分。

## 采集时间轴（本地时间）
| 时段 | 活动 | 文件 |
|---|---|---|
| 22:50:01–22:55:01 | **sit** in chairL（末尾起身） | `chairL_sit_20260713_225001.npz` |
| 22:55:02–23:00:02 | walk(22:55) + **lie**(22:56–22:59, D=3.76m) | `chairL_sit_20260713_225502.npz` |

采集：`cap_stream.py`（只读、滚动、每帧 `frame_ts` 墙钟 + `block_start_epoch`），18.78fps，profile_fall_20fps_gaze.cfg。
手表：采集时开 Workout("其他") → HR ~5s 一条 → 「导出所有健康数据」→ 抽出今天 = `watch_hr_0713.csv`。

## 干净窗（排除 22:55 walk 与起身）
- SIT: 22:51:00–22:54:00
- LIE: 22:56:20–22:59:00

## 结果（30s 窗，雷达盲 HR vs 手表，**sub-lag 插值 autocorr**）
| 段 | 胸部 bin | 雷达中位 | 手表中位 | **MAE** |
|---|---|---|---|---|
| SIT | 178 (4.17m) | 79.4 | 81 | 3.4 |
| **LIE** | 154 (3.60m) | 76.3 | 76 | **0.5** |
| **总计** | | | | **2.1 bpm** (n=30) |

图：`validate_watch.png`（上 SIT、下 LIE；橙=雷达插值,黑=手表,蓝=数搏动交叉验证,灰点=autocorr 整数-lag 栅格）。逐窗数据：`results_per_window.csv`。

- **整数-lag 栅格**（18.78fps: 70.4/75.1/80.5）会把真值量化，故用 **sub-lag 插值**脱栅格；LIE 脱栅格后 76.3±1 严丝合缝跟手表（非"卡死75"）。
- SIT 尾部 120–150s 掉到 70 = 22:54 起身前的运动污染，前 110s 贴合。
- r≈0 正常：静息 HR 近乎恒定，无轨迹可跟踪 → MAE 才是指标。
- 生理合理：sit 80 > lie 76（躺下 HR 降）。

## LIE 段真实性核验（回应"全是75是否假象"）
- 腹/胸**确实分开**：腹 bin163(3.81m,呼吸44μm) vs 胸 bin154(3.60m,呼吸16μm)，隔 **9 bin=21cm**。
- flat 75 是**真心跳**，三个独立方法收敛：FFT精细峰 **75.8**、时域数搏动 **75**、autocorr亚-lag插值 **76.7**（手表 76–79）。75.8=5.8×RR **非谐波**，非 band-center 伪值81。"全是75"只是整数-lag 栅格把 75.8 量化到 75.1。
- **必须分腹/胸**：腹部 bin 自估 HR = FFT 61 / autocorr 72（错），只有胸部 bin 给出真 76。

## 方法（真值-free）
demod 全 bin → 腹部=呼吸幅度最大 bin → f0 来自腹部 → **盲胸部 bin = body bin 中 cardiac-SNR 最大者**（[1.0,1.7Hz] 非谐波峰/噪底）→ HR = 该 bin 的 autocorr[1.0,1.7]。上游占用由 `living_gate` 负责（本目录未含）。

## 复现
```
.venv/bin/python3 validate_watch.py          # 出上表
# 手表重新对齐（若换文件）:
.venv/bin/python3 apple_hr_align.py <export.xml> --npz chairL_sit_20260713_225001.npz
```

## 文件
- `chairL_sit_20260713_225001.npz` / `_225502.npz` — 雷达 cube（44bin×~5640×16，含 frame_ts/block_start_epoch）
- `watch_hr_0713.csv` — 今天全天手表 HR（epoch,bpm）
- `watch_hr_sit_lie_windows.csv` — 仅 sit/lie 两窗的手表真值（wall_clock,bpm,segment）
- `validate_watch.py` / `apple_hr_align.py` / `chairL_blind_hr.py` — 验证/对齐/盲估脚本

结论记忆见 memory: `vitals-occupancy-gate`（HR-validity 已验证）、`apple-watch-hr-truth`、`next-crack-rr-harmonic`。
