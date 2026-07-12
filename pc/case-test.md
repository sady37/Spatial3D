# 采集/测试日志 (case-test)

每次雷达采集或现场测试记一行。字段说明：
- **时间**：采集日期+时刻（本地）
- **编号**：单字母/短代号（cube 文件名前缀）
- **pose**：座姿 sit / 侧身 sidesit / 躺 lie / 站 stand / 空场 empty / 运动后 post-ex …
- **距离**：人到雷达的距离 (m)
- **fps**：实测流帧率（采集脚本末行 `~Xfps`）
- **时长**：采集秒数
- **文件**：保存的 .npz
- **参照**：Apple Watch / 真值（有则填）
- **雷达 HR/结果**：`bcg_vitals.py` 或 `bcg_vitals_rt.py` 输出
- **备注**：几何是否变动、异常等

> 采集用法：`python cap_cube.py <out.npz> <秒>`（热流直采）。切 fps/窗须**先断电重启**再首份下发 cfg。
> 分析：`python bcg_vitals.py <cube> --fps <实测>`（单次）；`python bcg_vitals_rt.py <cube> --fps <实测> [--tachy 2.2]`（连续+房颤）。

| 时间 | 编号 | pose | 距离 | fps | 时长 | 文件 | 参照(AppleWatch) | 雷达 HR/结果 | 备注 |
|---|---|---|---|---|---|---|---|---|---|
| 2026-07-10 23:49 | sit39 | sit(正坐) | 3.9m | 18.8 | 240s | sit39_cube.npz | 80–85 | 81 (RT 中位 78.4) | 已验证基线 |
| 2026-07-11 00:02 | sidesit | sidesit(半侧身) | ~3.9m | 18.8 | 120s | sidesit_cube.npz | 83–91 | 81 (RT 中位 78.2) | 已验证；RR 侧身退化 |
| 2026-07-11 00:11 | lie41 | lie(躺) | 4.1m | 18.8 | 180s | lie41_cube.npz | 79–83 | 87 (RT 中位 76.8) | 已验证；躺姿 SNR 低 |
| 2026-07-10 22:48 | fall20 | lie+拉动 | ~4.1m | ~20 | 120s | fall20_cube.npz | — | 81 (RT 中位 80.1) | 跌倒/扰动测试 |

| 2026-07-11 18:0x | health | 空场/健康检查 | — | 18.8 | 15s | (scratch) | — | 44 bins 149–192, 282 帧 | reset 后 +30s 预热，20fps gaze cfg，流健康 ✓ |
| 2026-07-11 18:xx | emptyT | 空房(真负,无人) | — | 18.8 | 240s | emptyT_cube.npz | 无人 | 🔴→✅ 原始:单次 HR=120[HIGH]、AF 145/151 持续ALERT。**加占用门后**:151/151 NO PERSON、0 假警报 | 暴露"缺占用门"缺陷并已修复。判据=呼吸带位移 RMS(空房 1um vs 有人 ≥10um@15s,8× 干净分开);rr_spread 窗尺度无判别力已弃用。阈值 disp≥4um。四份有人 HR 基线全保持 |

<!-- 新测试追加到下方（保持雷达位置未动 = sidesit 几何） -->
