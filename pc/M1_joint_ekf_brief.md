# M1 — 双振荡器 EKF 联合解耦(呼吸-心搏),tachy2 单bin原型

**这是一个自包含任务 brief。** 目标:验证"联合状态空间跟踪"能把心率**稳定跟出趋势**,
解决前一轮所有单帧方法的通病——率不稳(autocorr 75 / beat 94-143 / Bessel 133 / bin65 113,
真值 131→91,全都跳)。完整前情见 `pc/NEXT.md` 顶部两节 + memory `tachy-miss-algorithmic`。

## 背景(为什么做这个)

- 毫米波心率的最大难点是**RR 谐波落进心跳带**(β_r 大 → 贝塞尔谐波梳到 n~12;tachy2 真 HR 早段
  128 落在呼吸 6/7 次谐波间,晚段 91≈5×RR 直接撞谐波)。单通道频域滤波治不了。
- 前一轮把 tachy2 从"假读 81"推进到"**能检出偏高**"(Bessel-comb 拟合+心跳超出量,`bessel_fit.py`,
  中位 133、阈值110 与静息零误报)。**但精确数值/轨迹没一个方法能稳**——共同病根是
  **率不稳 + "先减呼吸再找心率"的串行误差传播**。
- **联合解耦**(用户提的方向):把 x(t)=R(t)+H(t)+N(t) 作为**同时估计**的隐变量,靠
  **时间连续性 + 波形先验 + 多通道**把率稳住。M1 先做**单bin、双振荡器 EKF**,验证连续性+形状
  能不能把 f_H(t) 平滑跟成 131→91(而不是跳)。

## 模型规格

**状态** x = [φ_R, ω_R, a_R, φ_H, ω_H, a_H]
(呼吸/心搏的相位、角频率、幅度;可加 DC/漂移项)

**观测**(用相位!用户坚持"基于相位",且已验证 demod 相位是对的域):
```
y(t) = a_R·g_R(φ_R) + a_H·g_JKL(φ_H) + n(t)
```
- y(t) = chest bin 的相位解调位移(`demod_channels` 输出,见下)。
- g_R:呼吸波形。起步用**不对称波形**(吸快呼慢:吸气半正弦 + 呼气 e^{-t/τ},τ≈0.5-1s)
  或前 3-4 次谐波合成;文献(Pi-ViMo, arxiv 2303.13816)用 sin^k / 二次吸气+指数呼气。
- g_JKL:心搏波形。起步用**固定双/三相模板**(J 上-K 下-L 上),或从数据学(见 `beat_morph.py`
  的模板学习)。**关键:形状固定、频率变**(用户洞察)——正好用相位振荡器 g_JKL(φ_H) 表达。

**过程**:φ̇=ω(相位积分);ω_R、ω_H 慢随机游走(强制平滑漂移);a 慢随机游走。

**滤波器**:EKF 或 UKF(观测非线性)。若 EKF 因多模态锁错谐波 → 上**粒子滤波**。

## 关键:f_H 初始化(否则必锁呼吸谐波)

单纯 EKF 会把 f_H 锁到最强的呼吸谐波。**必须给 f_H 好的初值 + 强连续性**:
- 用 `bessel_fit.py` 的心跳超出量、或宽带估计给 f_H 初值(tachy2 ~130,tachy3 ~85);
- ω_H 过程噪声设小(HR 慢变),让连续性压住谐波诱惑;
- 可加软先验:f_H 远离 n·f_R 的整数倍(避免锁谐波)。

## 数据(fps=18.78)

| cube | 距离 | 真值(Apple Watch) | M1 期望 |
|---|---|---|---|
| `tachy2_cube.npz` | ~2.1m | **131→91**(0-60s 131→110;60-120s 109→91) | f_H 平滑跟出下降 |
| `tachy3_cube.npz` | ~2.2m | 84-87 正常 | f_H 平 ~85 |
| `sport33_cube.npz` | ~3.3m | 101→106→82 | 远距,退化对照 |

取 chest bin:
```python
from bcg_vitals import demod_channels, estimate_rr, bandpass, sqi, RR_LO, RR_HI
import numpy as np
d=np.load('tachy2_cube.npz',allow_pickle=True); cube=np.asarray(d['snapshots'],np.complex64)
counts=d['counts'].astype(int); bins=d['bins'].astype(int); C=cube[:,:int(counts.min()),:]
chans=demod_channels(C,bins)               # (nbin,T) mm 位移 = 相位解调
_,f0,_,_=estimate_rr(chans,18.78)          # 呼吸频率初值
rr=np.array([sqi(bandpass(c,18.78,RR_LO,RR_HI),18.78,RR_LO,RR_HI) for c in chans])
chest=int(np.argsort(rr)[::-1][0]); y=chans[chest]   # 单bin观测
```
(demod_channels 内部:每bin 均值波束 z=C[i]@mean.conj(),再 unwrap 相位 → mm 位移)

## M1 成功判据

- **tachy2**:跟出的 f_H(t) 从 ~125-131 **平滑下降**到 ~91-95(和真值 ±15bpm,**趋势/斜率对且平滑,不跳**);
- **tachy3**:f_H ~85 平;
- 和单帧基线(跳 75-143)对比,**连续性明显改善**即算 M1 成功。
- **不追每拍精度**(2-3m 是边缘信噪,M1 只验证"联合跟踪能稳住趋势")。

## 前一轮的坑(别重犯)

- chest bin 的心跳是**噪声底上 ~1.2× 小凸起**(见 `energy_map.py`),但**相位残差里 JKL 拍肉眼可见**
  (`beat_morph.py`/`phase_diag.py`);难在自动、稳定。
- **均值波束会压制心跳**;心跳空间可分性在**相邻 range bin(tachy2 是 bin65)**——M1 先单bin,
  M3 再上多bin/多天线。
- 角度体素分辨率太粗(29°),别指望角度分离(`voxel_vitals.py` 已证负)。
- 谐波不是 log 衰减,是 **Bessel 平台**(`harmonic_decay.py`)——波形模型别用 log/纯正弦。

## 相关文件

`bcg_vitals.py`(demod/estimate_rr/LAMBDA_MM=5)、`bessel_fit.py`(检测器+f_H先验)、
`beat_morph.py`(JKL 模板学习)、`peak_cycle_probe.py`(呼吸周期切分)、`NEXT.md`(全程日志)、
memory `tachy-miss-algorithmic`(物理根因+方法账本)。

## 交付

一个 `joint_ekf.py`:双振荡器 EKF,输入 cube+fps,输出 f_H(t)/f_R(t) 轨迹图,
在 tachy2/tachy3/sport33 上跑,报"能否平滑跟出 131→91 / 85 平"。成了进 M2(学模板)。
