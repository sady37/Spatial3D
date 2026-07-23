# Cube 跌倒判读流程

AWRL6844 fall pipeline。**cube 是唯一权威**;3001 只负责过滤与起 18s 钟,判决走 cube 的两维返回,trigger 无权在原地否决。

色标:🟢 已实现 · 🟠 TODO(设计定案,未实现) · 🔴 报警 · ⬛ 作废

> 交互版(深/浅色自适应):[`cube_fall_flow.html`](cube_fall_flow.html)

```mermaid
flowchart TD
    A["TI alarm — window-Z / MLP (TLV 321)<br/>—或— 3001 floor_fall(楼下云)"]:::src
    A --> B["down = (w_down AND real_person) OR floor_fall<br/>⚠ down 对遮挡/远身体是胡猜(可信度≈0)"]:::proc
    B --> C{"down 首次 = 1 ?"}:::dec
    C -->|"是"| D["起 18s 钟 · 3001-first"]:::done
    D --> E["等 18s<br/>0–18s 只 3001 过滤 · 不发 cube · 不报红"]:::done
    E --> F["cube query<br/>目标 = 楼下云 median bin<br/>门:fresh · 非busy · 速率 · 固件预算"]:::done
    F --> G["cube 一发返回"]:::proc

    G --> AB{"cube 校验<br/>(A) 位置:查询bin 与 当前楼下云bin ≤ 10 且存在(8e0cfdd)<br/>(B) 归属:回包必须属于【本次主动查询】· query-epoch 相等(0722d)<br/>发新查询即作废旧结果 · bin 距离判不了返回时间"}:::done
    AB -->|"任一失败"| VOID["作废 · 视为无返回"]:::discard
    VOID -.->|"30s 自动重查"| F

    AB -->|"通过"| P["① PRESENCE — lying (Y/N) 单独定 fall"]:::hd
    P --> P1{"cube_ff 够强?"}:::dec
    P1 -->|"强"| PY["lying = Y<br/>(cube_ff 主判据)"]:::done
    P1 -->|"<0.5 / =0"| P2["z40 兜底 · 已实现 3449523<br/>差值/基值 vs 空房 · XY 逐格<br/>(多簇 chair隔断 仲裁仍 TODO#2:现由 cube_ff<0.5 近似)"]:::done
    P2 --> P3{"z40 ≥ 0.4 ?"}:::dec
    P3 -->|"是"| PY
    P3 -->|"否"| PN["lying = N"]:::discard
    P1 -->|"0 条目 · z40 也无数据"| PX["作废 · 不评估"]:::discard

    AB -->|"通过"| L["② LIVENESS — Living_state(仅标签)"]:::hd
    L --> L1{"RR 或 micro 测到?"}:::dec
    L1 -->|"是"| LV["Living"]:::done
    L1 -->|"测不到 · 仅腿/胸被挡"| LU["? 未知<br/>≠ 崩溃"]:::todo

    PY --> Z{"Fall ≥ 1<br/>任一发 lying = Y ?"}:::dec
    Z -->|"是"| R["🔴 FALL · 报警<br/>Living → 红 · ? → 红 + 活体未知(非崩溃)"]:::alarm
    R --> T["确认后按住红(报警已完成)<br/>撤警 = 恢复 或 cube 连2次阴性 → 复位 fall + 3次预算<br/>恢复(距离无关三选一):近=cloud_up · 远=起身移动的GTRACK轨迹 · 或跌倒点云清空<br/>(cloud_up 远距失灵:站着整云中位掉<0.4;down-gate 作废=进cube已不可信)"]:::done

    F -.->|"无资源 / 空返回"| RT["每次跌倒 ≤ 3 次 query(确认@+18s → +60s → +60s)后停<br/>报警已完成 —— 红色由确认按住,不再刷新<br/>3×6s=18s/次 ≪ 固件预算(cubeGuard 300/300/3000)· 不 wedge、自终止<br/>人恢复(cloud_up)/ cube 连2次空 → 撤销 fall + 复位 3 次预算"]:::done
    RT -.-> F

    classDef src fill:#0d9488,color:#fff,stroke:#0b7d72,stroke-width:1px;
    classDef proc fill:#475569,color:#fff,stroke:#334155,stroke-width:1px;
    classDef dec fill:#cbd5e1,color:#0f172a,stroke:#64748b,stroke-width:1px;
    classDef done fill:#0e9f6e,color:#fff,stroke:#0b7a55,stroke-width:1px;
    classDef todo fill:#d97706,color:#fff,stroke:#a85d05,stroke-width:1px;
    classDef alarm fill:#dc2626,color:#fff,stroke:#991b1b,stroke-width:1.5px;
    classDef discard fill:#64748b,color:#fff,stroke:#475569,stroke-width:1px;
    classDef hd fill:#6366f1,color:#fff,stroke:#4f46e5,stroke-width:1px;
```

## 硬约束

- **18s** 3001-first:前 18s 只过滤,无 cube、不报红。
- 固件 cubeGuard 硬窗口 `300s`(3000 帧 @10fps)。
- 预算 `300` cube-帧/窗口(30s)= **10% 占空**;单发上限 `300` 帧(30s);server 用 60 帧/发。

## 判决原则

- **Fall ≥ 1**:任一发 `lying=Y` 即报。
- presence 主判据 = **cube_ff**(≥0.5),z40 兜底(cube_ff <0.5/=0/多簇 时)。
- `lying(Y/N)` 单独定 fall;`Living_state(Living/?)` 只贴标签。
- **"?" = 仅腿/遮挡测不到,≠ 崩溃。**
- **cube 是权威**:进 cube-query = down 已不可信 → down 不再排/撤 cube;确认后按住红。
- **恢复撤警(距离无关,三选一 OR)**:近=`cloud_up`(整云中位>0.4);远=**起身移动的 GTRACK 轨迹**(直立+speed>0.2,安全主力,静躺不产生移动轨迹);或**跌倒点云清空**(≤3点,保守防误撤远-稀疏真躺)。持续 RECOVER_S 才撤。⚠️ cloud_up 远距(>3m)失灵——站着整云中位掉 0.0-0.4、和躺重叠,故必须补 B/A。

## 阈值(已定案,用 case/ 标注数据标定)

- **cube_ff = `0.5`**:≥0.5 用 cube_ff 判 lying;<0.5 转 z40。(躺好信号 0.55-0.92 vs 远/静止 0.00,双峰空档)
- **z40 = `0.4`(现有,不动)**:down 已成立,只判躺(~28)vs 空(~0);站/走上游点云 Z 已排,不用抬。
- **每次跌倒 ≤ `3` 次 query(报警完成模型)**:确认@+18s → +60s → +60s,然后停 —— cube 的活是"确认这次跌倒 + 刷两次呼吸",报警一响就完事,不无限刷。红色由确认按住,**人恢复(cloud_up)即撤销 fall 并复位 3 次预算** → 下一次真跌倒重新拿 3 次。3×6s=18s/次 ≪ 固件预算,不 wedge、自终止(避免无限 query 卡死/恢复漏检卡红)。
- **cube 校验 = (A)位置 `10 bin`(1bin≈10.8cm → ~1m)+ (B)归属 `query-epoch`**:发起查询即 +1,回包打戳,判决只认 `epoch == 当前` → 发新查询立刻作废旧包(fall1 的 cube 永远确认不了 fall2,bin 距离判不了返回时间)。
- ⚠️ cube 波束宽 → 分不了姿态/家具;姿态=点云 Z,排家具=z40+一次性空房基线。

## 状态

| 已实现(commit) | 内容 |
|---|---|
| c1110ac / b1f1adf / 8982ea6 / 7fb173c | 基线:cube 目标=楼下云 median · z40 dr+XY逐格+堵红漏 · 删 far-force · 确认后锁红 |
| 3449523 | cube_ff 主 / z40 兜底(纠正 A+B z40-primary 弄反) |
| eaafa5f | 重试刷新去停查自锁(确认后仍刷新) |
| 8e0cfdd | cube 校验 (A)位置 10 bin |
| 0722d | (B)归属改 **query-epoch** 时序绑定(替换 resp_bin ±10):只认本次主动查询回包 |
| 52e1f21 | 重试节奏 60→30s(临时;已被 0722e 覆盖) |
| 0722e | **报警完成模型**:每次跌倒 ≤3 query(@+18/+60/+60s)后停;回 60s;恢复(cloud_up)撤销 fall + 复位预算 |

| TODO(未实现) | 内容 |
|---|---|
| #1 | down-gate:只认 down 为真时抓到的 cube(TODO1 3001 veto 重设计的时序补) |
| #2 | 真·多簇 per-cluster cube 分裂仲裁(现由 cube_ff<0.5 近似) |
| — | RANGE_STEP 校正(0.085 vs 实测 10.8cm/bin) |
