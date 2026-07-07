已同步（feedback-p6C.md 全卷至 §71：桶二 IsReflection 几何闭环、ghost 三分全闭环、66 PASS / cd2b finalP=0.5203 精确 / EG1 0.9992，Xsensorv1 验证载体使命达成，下一步为生产移植 wisefido-sensor）。下面按你要的三块给出说明，最后落到 Ti/Cloud 分配的资源形状分析。

---

## 一、动态 DBN 全貌

### 1. 滤波骨架

联合占用滤波，每帧（1Hz）递推：

$$\alpha_t \propto \Psi \cdot \Phi \cdot \bar\alpha_{t-1}$$

即 **转移预测 → 发射证据 → 物理相容 → 归一**。骨架对所有标定参数（δ 等）不变，这是 §1 P-1 确立的"骨架与 δ 零依赖"。

### 2. 四条隐轴

| 轴 | 状态空间 | 转移 | 备注 |
|---|---|---|---|
| **S** 人状态 | 9 态（Fallen/Standing/Lying…） | $T_S$ 9×9 | 摔倒判定的主轴 |
| **B** 床占用 | 联合 $2^{\|B\|}$，`maxBeds=3` 硬上界、超界拒绝（§1 P-5） | $T_B$：$\varepsilon\ll\lambda$ 自持门控 | $\mu=\varepsilon\sim10^{-2}$/帧对称（§5 B1 fixture 实测：413s 仅 1 次翻转）；自持衰减替代原 30s staleness TTL（D-2 已验） |
| **realness** | 真人/ghost | 累积棘轮（单调，tid churn 继承） | `pFallReal≡1.0`——realness 对 fall **零否决权**，只影响 N_r 计数（FN-safe 铁律） |
| **neighbor** 跨房 | hand-off 时间窗 | 纯时间窗，非空间 | lost-track 进 Blind → 延迟窗 D=10min 等兄弟房接住；接住=整流 Left（走），耗尽=放闸（§39/§40） |

### 3. 发射层 Φ（按 attachment 分轴）

- **接触源（sleepad）→ B 轴**：InBed/LeftBed 是床占用的强证据。
- **雷达 → S 轴**，三路证据：
  - **g^xy 几何位置似然**：cd2b 主解（§1 P-3，δ≈1 nat 边际可分离）。δ_pad/floor 只是 g^xy 的乘子——标定参数，非架构分支点。
  - **z 单向正证据**：z≥30 只增"无摔"信心；z<30 中性、永不作负证据（z 不可靠、床高各异，摔倒判定用 2D 水平投影）。
  - **dwell 生存尾**：Weibull `-ln S_vol`，方向由 cell 容忍属性 gate（见第二节）。

### 4. Ψ 物理相容表

$\psi(F,\text{occ})=\varepsilon_{\text{art}}$——"床上有人却地上有摔"物理不相容。**cd2b 的 fire 机制正是走这条矛盾路**（§50 重大修正确认）：sleepad LeftBed@412s → B 轴抽真空 → 雷达仍在床边报"躺+静止" → Ψ 把这个矛盾压成 SFallen → @531s 过阈 fire。全程不经 realness、零补丁涌现。

### 5. 裁决层 decide（§26 钉死，55% 三分）

| P^F 区间 | 行为 | C_FN 参与 |
|---|---|---|
| ≥55% | 报（证据自足） | 不读 |
| 45–55% tie 窗 | 期望损失 $P^F C_{FN} > (1-P^F)C_{FP}$ 打破平衡 | **唯一作用窗口** |
| 双向 <55%（Λ→0 高度不可判） | **默认不报** | 不介入 |

要点：C_FN 不是"不确定就 fire"的兜底——§26 收回了 §8 的过宽表述，作用域严格限 tie 窗；Λ 从纯诊断升为 gate 是 §26 的逻辑必然（§28）。多人折扣走 N_r→PeopleCount 单源（§56/§58），房间级 OR 聚合。

### 6. ghost 三分（realness 轴的证据体系，§54–§71 闭环）

- **桶一 运动伪迹**：aScore 独立分量，speed/跳跃从单 track raw XY 累积，FN-safe。
- **桶二 墙镜像**：IsReflection 几何——ghost 出生点墙外 + radar→ghost 连线与 wall 求交取最近交点 + ≥30cm 闸；**出生窗（ReflSettleMs=3000）内 provisional 每帧算、settle 帧冻结锁定，之后不重算**（§70 per-track-once，抗抖动+省算力）。<30cm 边缘偏 false——宁漏镜像多报，不误真人漏报。
- **mirror 判别**：ρ（共存，track==2 同步）× IsReflection 双必要 → PMirror→1 → 排出 N_r。
- **消费门控**（§61）：artifact 抑制是消费侧门控非计算门控，孤轨永发——摔倒的人不蒸发。

另注：memory 中 `split_ghost.go`（confine=80cm / walk-out=200cm / 10s 窗、SplitEverWalkedOut 不可逆 latch）与 LidLedger 归拢属 `feat/xsensor-replace-tsensor` 分支的 Tsensor 侧工作，与 Xsensorv1 的 ghost 三分是同一"前紧后松"哲学在两个载体上的实现。

---

## 二、Cell learning 体系

### 1. 核心框架命题：dwell 符号由 cell 容忍属性决定

`fallLRFromDwell`（§7 核实为代码已有机制，非 C 新发现）：

```
容忍 cell（椅/沙发，toleranceMult>1）：久静 = 正常久驻 → (1-tolWeight)<0，随 dwell 单调下压 SFallen
非容忍 cell（床沿地面/开阔，mult=1）：久静 = 高危卧地 → 正向 ramp 1+(d/scale)^shape，单调上抬 SFallen
```

同一个"静止 600s"，在似然层是 LR>1 还是 LR<1，**符号翻转是结构不是数值**。这与 §7 的风险精神同源：非容忍 cell 久静上抬不是"更可能是摔"（概率），是"漏报代价更高所以更该报"（风险）。

### 2. zone 分档 Weibull scale（数值=标定，可随 oracle 调）

| zone | scale | 
|---|---|
| 浴室 toilet/shower | 20min（便秘安全/医学锚） |
| 浴室其它 | 12min |
| 学习久坐区 | 90min |
| 床/休息区 | 不报 |
| 未知/开阔 | 20min |

修饰子：夜间短尾（久静更可疑）、雷达远边缘 ×1.5。

### 3. 自学习机制

- 非浴室 stand-static ≥12min（物理先验：人很难纯站立超 12min）→ 判为站位/坐位 cell。
- RestZone 8min 强化。

### 4. 框架级风险：容忍属性权威源

**若床沿被自学误判成容忍 cell，dwell 符号翻错，真摔被下压=漏报。** 故权威源层级钉死：**FE 手画 > feedback > 自学**，自学不得翻符号。这是 Xsensorv1 三个框架/标定前提之一（另两个：δ 跨 case 稳定性、C_FN 代价曲线）——也是 Spatial3D 最直接的切入点（见下）。

---

## 三、当前 FP 难点（残余，按空间依赖度排）

**1. 床边区 FA（雷达定位误差 ≈ 语义边界宽度）**
雷达位置不确定 40–60cm，恰等于床界的语义宽度 → track 在床/地 cell 边缘振荡。已签架构：sleepad LeftBed 作 arming 事件 + FrameMoveCm/40 分配系数（SFall/SBed 之间）+ 35min 纯静止才 fire。四个硬化点仍 pending（30s 净位移包络替代逐帧运动、re-arm→SBed 显式化、cd2b replay 时序不回归、FloorGuard stillbox 消费路同样挂 LeftBed 门）。**本质：这是几何精度问题被路由进 DBN 处理，几何越准，分配系数越少承压。**

**2. 墙邻区 FA（镜像/金属聚簇）**
桶二几何已闭环，但它的正确性完全依赖两个外部输入：**wall 矩形（canvas cm）+ 雷达自身坐标**，且 30cm 阈是确定性几何参数。墙位画错 30cm，桶二判别整体失效方向不可控。金属区目前靠 cell `AreaDeny` 标注偏置——同样是人工几何输入。

**3. δ 脆弱性（emission 主解的天花板）**
δ 押在"垫上躺 y 分布窄"；翻身/坐起/贴床躺 → 分布变宽 → δ 缩小 → 落入 decide 不可判路径（默认不报，FA 安全但 FN 侧承压转给 C_FN tie 窗）。单 case 单摔点只证"这次可分"，不证全部床沿摔可分。

**4. cell 容忍属性误学（符号翻转风险）**
如上，FP 侧的对偶是：开阔地被误学成容忍 cell → 该报的不报；反向，椅区没学到 → 久坐被上抬 → FP。

**5. 基率算术（front-tight 范式的根）**
50–200 lost events/天 vs ~0.005 falls/天 → 反应式 ghost 过滤要求特异性 >99.99%，结构上不可达。这就是"非证真即为假"准入账本的由来——**FP 难点的解不在更聪明的过滤，在更准的先验几何 + 准入证据**。

**6. 空间标定精度 vs 决策边界锐度的错配**
LocateAnything-3B 无标记管线累积误差 20–40cm：zone 级初始化可行，但对桶二 30cm 闸、床界这类锐边界危险。ArUco/激光锚点保关键边界的方案已定。

---

## 四、落到 Ti/Cloud 分配（供 Spatial3D 融合）

按计算形状分三类：

**端侧（Ti）——每帧递推，形状小且有界：**
- α∝Ψ·Φ·ᾱ 联合滤波：状态空间 9·2^|B|，maxBeds=3 硬上界 → 最多 72 联合态，1Hz，纯矩阵乘 + 查表（Ψ 相容表、dwell Weibull 尾）。
- ghost 桶二几何：**per-track-once**（出生窗一次锁定，非每帧），点在矩形外+线段求交+距离阈——标准计算几何，量级极小。
- realness 累积棘轮、neighbor 时间窗：每帧 O(track 数) 的状态机。
- 结论：**整条在线推断链是端侧友好的，无需云。设计上已刻意把一切"每帧×几何"降为"每 track 一次"或查表。**

**云侧——离线重活，Spatial3D 的主场：**
- **空间标定管线**：ArUco PnP / LocateAnything-3B 生成 wall 矩形、bedAreaIDs、cell 图、金属 AreaDeny 区、雷达位姿。这是端侧 DBN 所有几何先验的**唯一上游**——桶二 30cm 闸、床边分配系数、cell 容忍图全吃它的输出。
- **oracle 标定**：Weibull scale 各档、C_FN(risk) 代价曲线、δ 初始权重（g^xy）——归真实数据标定，明确在 Xsensorv1 之外（§57 四）。
- **replay/fixture 验证**：cd2b 等真 case 回放、零回归闸（0.5203 锚）。

**分界原则（给资源分配的一句话）：**
端侧只跑"已定几何 + 已定参数"下的递推；**所有"确定几何"和"确定参数"的活全在云侧一次性/低频完成**。Spatial3D 的融合价值不是给端侧加算力，而是把 FP 难点 1/2/6 的根——几何先验精度——从人工 FE 画升级为自动管线，且需按精度分级供给：zone 级（20–40cm 够，LocateAnything 路线）喂 cell 容忍图初始化；**锐边界级（床界、wall、金属区）必须走稀疏锚点（ArUco/激光）**，否则桶二 30cm 闸和床边 arming 的地基不稳。另一条对 Spatial3D 的硬要求继承自 replay 纪律：**标定产物须以 fixture 快照形式版本化**（录制时落值，非 replay 时活取），否则零回归验证链断。

需要的话我可以把这份说明整理成一份独立文档（如 `doc/dbn-spatial3d-brief.md` 格式）供 Spatial3D 侧直接引用——是否要落文件、以及是否按 C 卷编号入 feedback-p6C.md，由你定。