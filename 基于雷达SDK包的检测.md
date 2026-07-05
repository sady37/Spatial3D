红外微波能检测到人的存在，雷达100%也能，雷达的问题是在于无法识别出人的姿态导致的误判， 这点上，红外微波并没有提高可信度，而且红外、微波（10Ghz)的频率更小，精度更差。地震仪才能真真补盲，因为信号与雷达的完全不同。

# 当前的局限
当前雷达的原始信号RAW I/Q 因为要走DVI接口，实际生产中无法实现通过MUSIC等高分辨率算法。所以只能用各雷达厂家SDK提供的接口，走UART获得数据。
当前SDK+UART的数据类型：TI mmWave SDK（或 Vital Signs / People Tracking Demo）在点云输出阶段（objectDetection DPC） 时，确实只返回“检测到的目标点，这些点是经过 Range-Doppler + CFAR + Clustering 等算法筛选后的： 
SDK认为“动态目标或人体反射”的点，不包括静态反射（如墙壁、床、家具）的点
原因：
1.静态物体反射信号在多帧中几乎不变，被算法视为 背景噪声 / clutter
2.TI 的 CFAR（Constant False Alarm Rate）和 clutter removal 逻辑会自动滤除这些静态背景
CFAR自动调整检测门限：识别雷达回波数据中的峰值（即潜在的点）
Clutter Removal（杂波抑制 / 背景去除）：帧与帧之间几乎不变的反射信号,可行

## 可用的解决办法：
### 方法1：关闭 clutter removal CFAR恒虚警率检测功能
mmwave_config.cfg  staticClutterRemoval 0
所有检测到的能量峰都会输出，包括墙、地板、床等固定物，但提高灵敏度必然会大幅增加噪声点和虚警，并不能区分静态结构数。

### 方法2：关闭 Clutter Remova
可配置的 Demo 中，.cfg   staticClutterRemoval <0/1>
People Tracking Demo                    可关闭
Vital Signs Demo                        不可关闭，除非修改源代码
3D People Counting / Overhead Demo      可关闭

所以采用方法2：关闭Clutter Remova,可用于空间建模。
因为People Tracking Demo/Vital Signs Demo 无法直接通过cfg直接修改，最佳方式：多镜像 + 外部MCU 控制切换。


# 空间建模方式：People Tracking Demo-A
该方案在TI AWR6844AOP雷达（4TX 4RX MIMO，60-64GHz）下可生成稳定室内空间模型（5cm精度，固定结构识别率>95%）。
## 算法采用
1.关闭Clutter Removal（保留静态点，如墙体/家具零多普勒反射）
2.预识别强度阈值过滤多径（SNR阈值k=0.8，针对铝/钢/镜面/电视机r表）
3.距离优先于反射率（range阈值δ=0.2-0.5m优先，SNR辅助确认变化<10%）
4.低门限CFAR，
5.低门距提高距离精度


预识别：预识别高反射材料区域（高SNR簇），然后剔除低于阈值强度且距离大于直接路径的点（视为多径反弹）
SDK DPU或PC后端DBSCAN聚类高SNR簇（>15dB for 金属/镜面），取簇内最短range点为“原始”（d, θ_0, SNR_d）。
过滤逻辑：对剩余点，若SNR < SNR_d * k (k=0.7-0.8) 且 range > d + δ (δ=0.2-0.5m，材料相关) 且 Δθ = |azimuth - θ_0| + |elevation - θ_0| < α (α=5°，分辨率15°/30°下)，剔除为多径。
单雷达优化：1分钟扫描累积验证“原始点”一致性（range变化<0.1m），后端Open3D实现<50ms/帧。电视机/镜面阈值调低（k=0.7），铝/钢调高（k=0.85）。
预期效果：在4x6m房间，金属多径去除率~70-85%，布局精度<5cm（结合下采样+聚类）。

距离优先于反射率：窗户区，拉帘/不拉帘， 反射率/信号功率不影响边界判断


## 优化 初次安装时采用双雷达校准
 双雷达+30cm水平基线+预置强度阈值 
针对已识别的金属区域（高反射率处），将低于阈值的强度点且距离大于原始直接路径距离的点视为多径干扰;
                单雷达：      A/B双雷达（2-3x）
高度误差（最远处4m）：2.31 m    ~0.8m （正是这个原因，导致床上/下识别失误）
横向误差（最远处4m）：1.07 m    ~0.4m

示例性能模拟（4x6m房间，最远处4m）
单雷达基准：横向误差~1.07m（4m × tan(15°/2)≈0.53m，但分辨率半宽~1.07m），高度~2.31m（4m × tan(30°/2)≈1.15m，半宽~2.31m）。
双雷达提升：融合后横向~0.4m（视差优化~2.5x），高度~0.8m（仰角融合~3x）。多径去除后，布局完整性>95%（金属墙角虚假点<5%）。

优势：30cm基线提供视差（parallax）改善远场深度精度~2-3x（最远处4m，横向误差从单雷达~1.07m降至~0.4m，高度误差从~2.31m降至~0.8m）。
预置金属反射率过滤（基于SNR阈值~10-20dB）可去除~70%多径（路径长衰减），结合低门限CFAR捕捉弱静态点（如家具）。多帧融合（3分钟~5400帧/雷达）确保完整性，适合墙装配置（前向4m，左右3m）。
局限：仰角分辨率~30°仍限高度精度，需RANSAC平面拟合辅助；计算开销~100ms/帧（主机PC处理）。
精度预期：整体布局误差<5cm（范围~4cm+融合优化），覆盖率>90%（双视角减少遮挡）。

不足：需要在安装时，额外增加一个雷达作初始测量。


## 后期优化 单雷达-1周每天2点建模（总14次扫描，1分钟/次，累积~42k帧）
>拉长周期，优化
>默认只计算固定区域，其它区域视为活动区域，如门，存在开/关/半掩，很难进计算，还是人工标注简单 
>每天凌晨2-3点：房间无活动人体时，启用空间扫描，并向IoT更新最新的空间布局。注意，要保存最近30天的资料。



# 动态扫描（People Tracking Demo模式） 
空间建模阶段聚焦于静态扫描，识别固定区域（墙体/家具等高反射不变表面）和活动区域（门/窗/人走动区），通过设置雷达信号的最大距离阈值（e.g., 房间边界4m），限制处理范围，便于后续多径过滤（只在固定区域内应用强度/距离阈值）。其他未识别区间默认标记为活动区域（无需细化）。

##  床上双设备检测 V1.5版本上要实现
状态A:当人体走向床边或质心在床区域时，持续5秒  T为真 F为假
状态B:人在床上超过5秒，雷达进入Vital Signs Demo 模式
状态C:人在床上，且姿态为躺
压力板：
状态H：压力板有呼吸/心率，上床检测：有或无
状态I:压力板检测到离床，15秒后，压力板呼吸/心率=0

判断压力板的可靠性：A&H  
    >当A=T，H=T,即雷达检测到人在床上，压力板的呼吸才为真时，否则可能是干扰，原因：雷达对人体轨迹检测的可靠性高
判断呼吸心率的可靠性：B||C且B>C
    >优先取睡眠板的值，当睡眠板无值时，用雷达的值， 呼吸、心率分别处理，即压力板仅丢失心率时，卡片上直接用雷达的心率。原因：压力板精度高
判断离床逻辑 C&I=T, 人跌落床下
    >当压力板检测到离床信号，雷达检测到人仍床上躺着，大概率是人从床上跌落躺在地上，此时雷达可能仍可检测心率。即使误报，也不影响


## 在人体信号突然消失 V1.5版本上要实现
在门或出入区域，视为走出，
其它活动区域：持续300秒仍未检测到，进入空间扫描模式，对比之前保存的静态背景，检测该区域变化率，超过30%即发送L2报警:可能跌倒或进入盲区，交由人类处理.
V1.5版本：持续300秒仍未检测到即发送L2报警:可能跌倒或进入盲区，交由人类处理.
附加：要不要起用雷达的VoIP及升级功能？这两项server上没有开发。


## 轮椅 暂不考滤

## 浴室金属把手干扰（这个可暂时不做）
目标：把手/支架静态后（velocity~0，确认无移动），过滤其高SNR反射（|Γ|~0.98），识别后方人体（不处理移动把手）。
### 1.cfg基础调优（静态杂波处理，启用Clutter Removal但弱化numAvgFrames=8）：

dynamicRACfarCfg -1 4 4 8 8 6 4 7.0 9.0 0.3 0（动态CFAR，discard窄4/4样本，thre低7/9捕捉人体）。
staticRACfarCfg -1 6 4 2 2 8 8 8 4 9.0 10.0 0.3 0（静态把手，discard 10样本~0.85m覆盖50-100cm长条，guard 8/4隔离后方）。
clutterRemoval 0 8（弱化平均帧，保留把手静态后快速更新）。

staticRACfarCfg -1 8 6 2 2 8 8 8 4 9.0 10.0 0.3 0 0
// subFrameIdx=-1（所有帧），discard range 14样本/angle 4样本，refWin 8/8，guard 8/4，thre 9/10。
staticRangeAngleCfg -1 0 8 2  // 启用静态处理（0=enable），azimDeciFactor=8（粗分辨率减MIPS）。

range维度参数（discardLeft/RightRange, refWinSizeRange, guardWinSizeRange, rangeThre）：

这些参数针对距离（range）bin处理静态/动态杂波，独立于把手方向。横放把手可能在range上产生线性延伸反射（长50-100cm覆盖~12-25样本），竖放则垂直堆叠（range变化小，但强度高）。配置中的discard 4/4（动态）/6/4（静态，总10样本~0.85m）和guard 6/4（动态）/8/4（静态，~0.7m）覆盖两者轨迹宽度（宽≤5cm偏移<0.2m）。rangeThre 7.0/9.0（低阈值捕捉人体）确保后方人通过，而抑制把手高SNR（>20dB）bin。
适用性：range不区分方向，横/竖把手均在相同range bin内反射（e.g., 2m处），配置同时隔离。


angle维度参数（discardLeft/RightAngle, refWinSizeAngle, guardWinSizeAngle, angleThre）：

angle参数在2D angle-range图（方位角azimuth + 仰角elevation）上操作，处理反射偏移。横放把手主要影响azimuth（水平偏移<3°），竖放影响elevation（垂直偏移<5°，仰角分辨率30°下）。配置中的discard 2/2（总4样本~3°）、guard 4（动态）/4（静态）和angleThre 9.0/10.0同时覆盖两者（angle域统一阈值，抑制<5°窗口内高SNR杂波）。
适用性：SDK的angle CFAR不区分azimuth/elevation子维度（2ndPass处理整体angle），配置通用。横放窄方位反射、竖放窄仰角反射均被guard窗隔离。


clutterRemoval numAvgFrames=8：

这弱化静态杂波平均（默认32帧），快速更新把手移除后bin（1-2帧响应）。横/竖把手静态反射（Doppler=0）均被初步抑制，但numAvgFrames低确保动态人体（velocity>0.1m/s）不丢。方向无关（Clutter Removal基于样本平均，非几何）。
适用性：平均帧数针对时间域杂波，独立于空间方向。


### 2.把手静态确认 & 过滤-一次性的：

#### 事件检测：post-CFAR点云中，velocity<0.1m/s + SNR>18dB + 线性簇（长度0.5-1m，宽<0.05m）=把手静态。移动时（velocity>0.1m/s）忽略（视为动态杂波）。
#### 清洗逻辑（PC后端或自定义DPU process）：
    距离优先：把手range_d后，discard range_d ±0.2m bin（覆盖宽≤5cm反射）。
    SNR辅助：剔除SNR >18dB + Δθ<3°的残留点（鬼影）。
    后方人体：range > range_d + 0.1m + velocity>0.1m/s + SNR 10-15dB确认（e.g., 胸腔微动）。

#### 伪逻辑：
    if velocity_handle < 0.1 and cluster_length ~0.5-1m: static_handle; discard (range_d ±0.2m); detect_human if range > range_d + 0.1 and velocity_human > 0.1。


### 3. 实时检测动态切换逻辑
  在浴室区（ROI预标注的空间模型）首次检测到人体后，引入2秒冷却期（避免瞬态误触发，如水溅或短暂反射），确认再次检测到人体（持续存在）才启用过滤逻辑（CFAR高抑制 + 多径阈值，针对金属把手/支架）；人体走出浴室区（质心离ROI>阈值）后，立即关闭逻辑（恢复低抑制配置）。

  手动标注： 
   <空间建模阶段>启用：
   生成2D平面模型，手动标注浴室区，进入该区域时启用过滤逻辑。这是一种“区域激活”模式，简单可靠（无需实时聚类），精度>90%。
    >生成2D平面：投影3D点云到XY平面,输出占用图（2D网格，5cm分辨率）。
    >人工标注：用Visualizer或Python Matplotlib工具，标记浴室区ROI（e.g., 矩形[x1,y1,x2,y2]，宽1m、高2m，覆盖把手/支架可能位置）。标注基于固定墙体边界（RANSAC拟合），JSON输出：{"bathroom_zone": {"bbox": [1.5, 2.0, 2.5, 4.0], "filter_enable": true}}。
    >集成多径过滤：标注区预设r表（玻璃|Γ|~0.4-0.8 + 金属条0.95-0.99），距离优先阈值δ=0.2m。
  
  <动态扫描阶段（People Tracking）启用：
    >区域检测：每帧点云投影到2D模型，计算质心（centroid）若落入浴室ROI（e.g., IoU>0.5），自动启用过滤（CLI或ioctl）。
    >（可选）实时聚类点云（简单DBSCAN-like，eps=0.05m），检测条状簇（形状阈值：长度/宽度比>10）。
    > 若确认条状金属（SNR>18dB + velocity~0），自动ioctl_setParam切换CFAR参数（e.g., discardLeft/RightRange从4/4扩大至8/6，rangeThre从7.0升至9.0），抑制该range bin。
    >过滤逻辑：进入区时，staticRACfarCfg切换高抑制（discard 8/6，thre 9.0/10.0），把手静态后（Δrange<0.1m）恢复（CLI sensorStop; ... ; sensorStart）。post-CFAR额外阈值：discard ROI内高SNR线性簇（长度0.5-1m）。
    >CLI自动化：用Python脚本监控ROI（serial接收点云），触发CLI（e.g., subprocess.call("sensorStop; staticRACfarCfg ... ; sensorStart")），延迟<100ms。
    每帧<50ms

# 关键资源：
Ti AWR6843AOP 3T4R分辨率准确性（TI AWR6844AOP雷达需要查手册）

方位角分辨率（Azimuth）15°：准确（典型值）。基于MIMO虚拟阵列和辐射图案（H-plane），半功率波束宽度（beamwidth）约15°，支持~90° FOV检测。E2E和MathWorks配置示例确认Azimuth 15°。
仰角分辨率（Elevation）30°：大致准确（典型~29-30°）。E-plane辐射图案显示beamwidth ~29°（视频参考设计），ODS阵列可优化至更高elevation但牺牲azimuth。实际取决于chirp配置和算法（如DBSCAN角度估计）。
范围分辨率~4cm：准确。基于4GHz连续带宽（60-64GHz），理论ΔR = c / (2B) ≈ 3.75cm（c=3×10^8 m/s），IF链支持10MHz带宽。实际~4cm（含噪声）。

总体，规格大致准确（角度/范围），但TX/RX为3T4R而非4T4R。若需4TX，考虑AWR系列自定义板或AWRL6844（类似规格）。若具体应用场景，提供chirp配置可优化分辨率。

视频演示：Intelligent Fall Detection Using TI mmWave Sensors（2019年发布，展示实时检测）。
https://www.ti.com/video/6074838932001
PDF参考：Non-contact and Private Stance Detection with TI mmWave Sensors（包含点云截图和算法描述）。
https://www.mouser.com/pdfDocs/Non-contact_and_Private_Stance_Detection_with_TI_mmWave_Sensors.pdf
E2E论坛讨论：Chirp Config for Stance Detection（2019年，提供chirp配置文件示例，用于身高检测和跌倒实验）。
https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/856139/iwr6843isk-chirp-config-file-for-stance-detection






这是一个结合了您所有优化和需求的**完整、夜间静态空间建模**方案。该方案使用 **$20\text{cm}$ 高分辨率**，并通过 **$\text{UART}$ 高速传输**和**长时间累积**，最大限度地利用了 $\text{TI AWR6844AOP}$ 的有限资源。

---

## 🌃 最终方案：高分辨率 $3\text{D}$ 静态分层建模

### 🎯 核心目标与配置

* **空间范围:** $4\text{m} \times 6\text{m} \times 3\text{m}$。
* **体素分辨率:** $L=20\text{cm}, W=20\text{cm}, H=20\text{cm}$。
* **体素总数:** $20 \times 30 \times 15 = \mathbf{9,000}$ 个。
* **工作模式:** **无运动**时运行 (夜间/无人)，使用 **16 个 $\text{Chirp}$** ($\text{SNR}$ 鲁棒性高)。
* **输出策略:** **每 $T$ 分钟** 高速 ($\sim 1 \text{ 秒}$) 传输一次完整的 $\text{9,000}$ 体素地图。

---

### 阶段 I: $\text{AWR6844AOP}$ ($\text{DSP}$ 核心) 任务

$\text{DSP}$ 核心负责**信号处理、运动过滤**和**体素累积**。

#### A. $3\text{D}$ $\to$ 体素分配与累积

1.  **静态点过滤 ($\mathbf{C66x}$ 优化):**
    * 在 $\text{Range-Doppler}$ $\text{Map}$ 阶段，执行 $\text{Doppler} \approx 0$ 过滤，快速排除所有动态点。
2.  **体素映射 ($\text{LUT}$):**
    * 使用 $\text{MSS}$ 预先计算并提供的 **查找表 ($\text{LUT}$)**，将 **静态峰值** 的 $(R, \theta_{\text{az}}, \theta_{\text{el}})$ 快速映射到 $\mathbf{9,000}$ 个体素索引 $(\mathbf{i}, \mathbf{j}, \mathbf{k})$。
3.  **增量式 $\text{OGM}$ 更新:**
    * 对被击中的体素 $\mathbf{V}[i, j, k]$，执行 $\text{Log-Odds}$ 增量更新。
    * **更新 $5$ 个统计字段:**
        $$\mathbf{V}[i, j, k] \leftarrow (\mathbf{L}, \mathbf{\Sigma I}, \mathbf{C}, \mathbf{Z}_{\min}, \mathbf{Z}_{\max})$$
4.  **$3\text{D}$ $\text{Ray}$ $\text{Tracing}$ (可选优化):**
    * 对 $\text{Radar} \to \text{Hit}$ 路径上的空闲体素执行 $\text{Log-Odds}$ 衰减 $(\text{Free})$，以提高地图精度。

#### B. 运动检测与控制

* **并行检测:** 持续运行 $\text{Doppler}$ $\text{FFT}$ 寻找**非零速度**峰值。
* **控制标志:** 如果检测到运动 (如人或动物)，设置一个**运动中断标志**。

---

### 阶段 II: $\text{AWR6844AOP}$ ($\text{MSS}$ 核心) 和 $\text{UART}$ 传输

$\text{MSS}$ 核心负责**高层控制**和**高速传输**。

1.  **工作流控制:**
    * **If ($\text{Motion}$ $\text{Flag}$ $\text{is}$ $\text{Set}$):** 发送 **“运动中断” $\text{TLV}$** $(\text{Type } 2001)$，通知外部主机**暂停建图**。
    * **Else (无运动):** 检查是否到达**定时传输点** $(\text{例如每 } 5 \text{ 分钟})$。
2.  **高速地图传输 (每 $T$ 分钟):**
    * $\text{MSS}$ 将 $\text{9,000}$ 个体素 $(\mathbf{V}_{3D})$ 的数据结构**分块打包**。
    * 使用 $\text{UART}$ 的 $921600 \text{ bps}$ 带宽，在约 **$\mathbf{3 \text{ 秒}}$** 内将**完整 $180 \text{ KB}$ 地图**传输给外部主机。
3.  **$\text{TLV}$ 数据封装 (自定义 $\text{TLV}$ $\text{Type } 2000$):**
    * 每个 $\text{TLV}$ 数据块包含：起始体素 $\text{ID}$, 连续体素数量, 以及 $N$ 个体素的 $5$ 个统计字段 $(\mathbf{L}, \mathbf{\Sigma I}, \mathbf{C}, \mathbf{Z}_{\min}, \mathbf{Z}_{\max})$ 的压缩数据。

---

### 阶段 III: 外部主机 (PC/MPU) $3\text{D}$ 分析

外部主机负责复杂、计算密集型的建模和识别。

#### A. 静态 $3\text{D}$ 建图 ($\mathbf{Walls / Furniture}$)

1.  **地图存储与更新:** 接收 $\text{UART}$ 数据，在本地维护一个高精度的 $\mathbf{V}_{3D}$ 累积地图。
2.  **墙体提取 (平面拟合):** 运行**$\text{3D}$ $\text{RANSAC}$** (或优化的霍夫变换) 算法，寻找地图中 $\text{LogOdds}$ 高的体素点集：
    * 识别 **垂直平面** $\rightarrow$ **墙体**。
    * 识别 **水平平面** $\rightarrow$ 地面/天花板。
3.  **家具识别:** 运行**连通区域分析**或**形状分类**，将 $\text{RANSAC}$ 拟合后的**残余体素块**识别为桌子、柜子等大型家具。

#### B. 地面层细化与高级识别

1.  **地面层提取:** 从 $\mathbf{V}_{3D}$ 中提取 **$Z$-索引 $\mathbf{k}=0$ 和 $\mathbf{k}=1$** (对应 $\text{0cm}$ 到 $\text{40cm}$ 高度) 的体素，作为**高精度地面背景图**。
2.  **镜子/金属识别 (基于 $\text{Intensity}$):**
    * 在墙体 ($\mathbf{W}$) 或家具表面 ($\mathbf{F}$) 的体素中，筛选出 $\mathbf{\overline{I}} = \mathbf{\Sigma I} / \mathbf{C}$ **超过阈值 $T_{\text{Specular}}$** 的体素块。
    * 结合 $\text{3D}$ 几何模型，验证这些高强度区域是否存在**镜面虚像**。
3.  **跌倒检测参考:** 将 $\mathbf{V}_{3D}$ (或其子集，如地面层) 作为**背景基线** $(\mathbf{V}_{\text{background}})$ 存储，以备在运动中断发生时，进行 **60 秒局部体素数据** $(\mathbf{V}_{\text{event}})$ 的高精度对比分析。