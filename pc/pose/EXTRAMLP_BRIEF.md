# ExtraMLP — "地面上有没有人 lying" 二分类  (design brief, 2026-07-19)

## 任务 (one line)
给定一个**位置**（由 TI 触发指出）的 **18 s 3001 点云 + 18 s cube(320)**，输出**该位置此刻有没有一个人躺在地面上**（yes/no）。**判「状态」不判「动作」** —— 不关心之前有没有 fall 的下坠动作。

## 为什么 (架构)
两个 cubeQuery 触发都来自 TI（`poses={tid:Pose}`，per-track 片上腿，与 server 的 boxes/prim 无关）：
- **① TI 报 fall/lying**：扫 `poses` **全 tid** 的 winDown / MLP Falling-Lying（不只 prim）→ 查该 track 的 bin。
- **② TI track 突然丢失**：用**最后一笔坐标**（云无关，远躺体的 3001 会塌成 0）→ 查该坐标 bin。
- 触发只指出「查哪个位置」；**ExtraMLP 用采到的 18 s 3001 + 18 s cube + 特征，判该位置有没有人躺着**。
- `clean.py` 现有硬规则（`RR AND floor_frac>=0.7`）是 ExtraMLP 落地前的**临时占位**；ExtraMLP 取而代之。
- ⭐ winDown-before-loss / ffrac / RR 阈值都是**置信度特征，不是否决门**。

## 判别的物理逻辑（决定特征设计）
| 情形 | 3001 高度 | 3001/cube 微动 | cube RR | 结论 |
|---|---|---|---|---|
| **躺地人**（正） | 低（次高 z≤~0.3）| 有 | 有 | **报 (yes)** |
| 站/坐的人（负） | 高（次高 z>~0.5）| 有 | 有 | 是人但没躺 → no |
| 空房 / 家具（负） | — / 静止 | 无 | 无 | 不是人 → no |
| 走开了（负） | 无点 | 无 | 无 | 那里没人 → no |
| **远躺体隐身**（正，最难） | **3001=0（不可见）** | cube 微动 | **cube 有 RR** | **报 (yes)** — 只有 cube 能救 |

→ **必须同时用 3001（高度：躺 vs 站）+ cube（RR：人 vs 家具）**。3001 分「躺/站/空」，cube 补「人 vs 家具」+ 救隐身远躺（002000 验证：bin51 RR 15–18 = 躺地活人）。

## 特征向量（per 位置 per 18 s 窗口）
**3001 (18 s @位置):** 次高 z、中位 z、floor-band 占比、点数/密度随时间、逐帧点变化(微动)、XY 展布 / flatness(躺=平铺)、驻留持续、3001-空(隐身标志)。
**cube (18 s @bin):** RR(0.15–0.5Hz) + strength、微动 band-frac、slow-time 方差/能量。
**上下文特征:** 地面 range(近/远)、was-winDown-before-loss(置信)、track-lost vs live、cloud-present vs 隐身、prim 有无。

## 输出
二分类 `person_lying ∈ {0,1}`（+ 概率/置信度）。**注意**：cube RR 分不了「躺 vs 站」（per-range-bin 无高度）——「在地面」的证据来自 **3001 高度**或 **TI winDown**；cube 主要确认「是人 vs 家具」。

## 标签
per-**窗口**（不是 per-录制）：一段 fall 录制里，站立阶段=负、躺地阶段=正。用物理云高做自动标签（`f_height<0` 持续 = lying，非循环），复用 `pc/pose/scene_features.py` 的 extract 思路（驱动真 `_scene()`）。⚠️ 远躺仰角上浮 → 自动标签用**每簇 floor_frac + 次高 z**，不要用中位 wz（会飘成 fallen~0）。

## 可用训练集（scene-format，有 ts+t_pose+cube）
**正样本（fall，躺地阶段有 cube）** — 单位：dur / 320-entries：
- fall_215500 227s/7560（近，clean）、fall_222000 104s/4011（近）
- fall_222500 278s/9345（3 摔，含远 4.5m）、fall_231000 242s/2471（GTRACK-drop 远）
- fall_231500 210s/6279（两远 4.5m）、fall_000000 248s/5880（近+远 4.2m）
- fall_013500 260s/2100（5 摔，含胸梗半跪）、fall_213500 197s/2520
- ⭐ record/live_scene_190500 199s/2100（**多人**：坐着的 + 远摔者）
- ⭐ record/live_scene_230500 177s/1260（远摔→走→近摔）
- ⭐⭐ record/live_scene_002000 157s/2016（**远摔久躺，live cube RR 15–18 确认**，golden）
- live_scene 191500/192000 300s/2100、192500 110s/2100、172000 68s/1680
（注：case/fall_000000==live 000000、case/fall_013500==live 013500，去重）

**负样本（无躺地人）:**
- two_seated_20260716_1125 300s，**0 cube**（2 人坐着）
- live_scene_214000_2 83s，**0 cube**（远摔但固件没吐 320）
- live_scene_191000 300s，仅 42 cube
- fall 录制的**站立/走动阶段**（同一录制内，per-窗口取负）

## ⭐ 核心数据缺口（必须补采）
**cube 只在触发（摔倒）时发 → 正样本(躺地)cube 充足，但负样本几乎没有 cube：**
1. **空房 / 家具 + 强制 cube**：现有 empty_/emptychair_/chairL 全是**旧 schema（无 ts/cube）**，无法 replay。
2. **站立/坐着的人 + cube**：正常录制里站着不触发 cube → 站立阶段无 cube。
3. **走开的人（track 丢失但那里没人）+ cube**：触发② 的关键负样本。

→ 需要**定向补采**：让 server 在「空房 / 家具 / 站立人 / 坐着人 / 人走开后」的位置**强制 cubeQuery**（可加一个 debug CLI/开关，绕过触发直接查指定 bin），采到这些负样本的 18 s cube。否则 ExtraMLP 只学了正样本、分不出「站着的人/家具也有 RR」。

## 落地路径
1. 扩 `scene_features.py`：per-触发-窗口 提取上表全部 3001+cube+上下文特征 + 自动标签 → 数据集。
2. 补采负样本（强制 cube 空房/站立/家具/走开）。
3. 训练小模型（logistic / GBDT / tiny MLP，可解释，小数据），LORO-CV（留一录制）。
4. 用 `fall_replay.py`（code-of-record）验证；触发①② 重新实现为**数据采集器**（喂 ExtraMLP），不做硬规则判定器。

参见 [[next-multiperson-percluster]]（两触发架构 + live 验证）、[[enhanced-scene-fusion]]（scene_features 脚手架）、`ENHANCED_MLP_BRIEF.md`（旧的场景融合 brief）。
