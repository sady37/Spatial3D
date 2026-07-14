# Radar Vitals — 实时网页框架

**显示与计算分离**：算法全在 `radar_pipeline.py`（纯函数），网页只渲染 JSON。
同一个 `analyze()` 既跑实时传感器、也跑录制 cube（对手表验证），换算法只动 pipeline。

## 四层
| 文件 | 层 | 职责 |
|---|---|---|
| `radar_pipeline.py` | **计算** | 纯函数：窗口→{present, fall, RR, HR, XYZ, 可测范围}。复用 `living_gate` / chairL 胸端簇 HR / MUSIC。可离线单测。 |
| `radar_source.py` | **采集** | 统一接口 `window(win_s)`。`ReplaySource`(npz@watch.csv 实时回放) / `LiveSource`(RadarSession 只读滚动缓冲)。 |
| `radar_server.py` | **服务** | stdlib http.server，`/api/state?bin_lo=&bin_hi=`，仅显示，绑 127.0.0.1。 |
| `dashboard.html` | **前端** | 可测范围 + present/fall/RR/HR(对比手表) + 俯视/侧视 XYZ + bin 位置滑块。 |

## 运行
```bash
cd pc/web

# 1) 实时（传感器需已在 stream；未 stream 先 radar_start.py）
#    录制是服务的固有部分：live 默认就录，落 pc/record/live_<5min桶>.npz。永不丢数据。
python3 radar_server.py live                            # 显示 + 默认录制到 record/live_*
python3 radar_server.py live --record demo              # 自定义前缀 -> record/demo_*
python3 radar_server.py live --no-record                # 仅显示，不录（少用）
python3 radar_server.py live --tilt 35 --mount 2.0      # 挂载标定 → 真实高度Z/pose

# 串口只能开一个：跑 live 时不要再单独跑 cap_stream（录制已合并进来）。按墙钟 5-min 桶
# (:00–:04, :05–:09 …)各存一个文件 pc/record/<prefix>_<桶起始>.npz(cap_stream 同格式)。
# 停止(Ctrl-C)会把当前未满的桶也 flush 落盘。
#
# 目录约定：pc/record/=原始录制,不上传(gitignore);  pc/case/=算法验证所需,上传。
# 确认某个 cube 要用于验证,再 `cp pc/record/xxx.npz pc/case/` 并提交。

# 2) 回放验证（无硬件，对手表逐窗对比）
python3 radar_server.py "../case/chairL_sit_20260713_225001.npz@../case/watch_hr_0713.csv"

# 浏览器打开 http://127.0.0.1:8765
```

## 功能对应需求
1. **可测范围 X,Y,D**：`/api/meta` 的 `range`，前端俯视/侧视画出覆盖锥。
2. **自动检测有人 / RR**：`living_gate` 呼吸相干占用门控 + `estimate_rr`。
3. **HR**：chairL 验证过的 **RR-锚定胸端簇 + 插值 autocorr**（手表 MAE 1.5）。
4. **实时对比手表**：回放时 `watch_hr` 逐窗对齐，前端显示 radar / watch / Δ。
5. **调整 bin 位置**：滑块设 `bin_lo/bin_hi` 强制 HR 搜索窗，实时看算法反应；空=全自动。

## 诚实边界（勿当成已解决）
- **倒地/高度 Z 依赖真实挂载标定**（`--tilt --mount`）。不给标定时 Z 不算、姿态显示"未标定"、**不误报 FALL**——因为 `to_room` 默认 tilt35°/2m 不匹配任意机位会把人算到地下。D(距离) 和 X(横向方位) 与挂载无关，始终可靠。
- **HR 是静息方法**。逐窗瞬时值比 README 的时间中位 MAE 1.5 更抖（单窗 SNR 有限）；看趋势对手表，别看单点。tachy/动态 on-harmonic 仍未破（见 memory `next-crack-rr-harmonic`）。
- **倒地检测 v1** 只是高度阈值启发式；正式方案见 memory `fall-detection-design`（静态3D能量密度），本框架预留接口未接入。

## 验证记录（2026-07-14, 回放 chairL_sit）
present=True，chest_bin 收敛到 **177-179**（与 chairL 验证一致），HR ~74-78 vs watch ~81-84（起身后过渡段）。bin 强制腹端→无干净HR，强制胸端→选到 bin178。倒地正确门控为"未标定"。
