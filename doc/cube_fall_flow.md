# Cube 跌倒判读流程

AWRL6844 fall pipeline。**cube 是唯一权威**;3001 只负责过滤与起 18s 钟,判决走 cube 的两维返回,trigger 无权在原地否决。

色标:🟢 已实现(本会话提交) · 🟠 TODO(设计定案,未实现) · 🔴 报警 · ⬛ 作废

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

    G --> P["① PRESENCE — lying (Y/N) 单独定 fall"]:::hd
    P --> P1{"cube_ff 够强?"}:::dec
    P1 -->|"强"| PY["lying = Y<br/>(cube_ff 主判据)"]:::done
    P1 -->|"弱 / =0 / 多簇(chair隔断·径向或轴向)"| P2["z40 兜底 · TODO#2<br/>差值/基值 vs 空房 · XY 逐格"]:::todo
    P2 --> P3{"z40 ≥ 0.4 ?"}:::dec
    P3 -->|"是"| PY
    P3 -->|"否"| PN["lying = N"]:::discard
    P1 -->|"0 条目 · z40 也无数据"| PX["作废 · 不评估"]:::discard

    G --> L["② LIVENESS — Living_state(仅标签)"]:::hd
    L --> L1{"RR 或 micro 测到?"}:::dec
    L1 -->|"是"| LV["Living"]:::done
    L1 -->|"测不到 · 仅腿/胸被挡"| LU["? 未知<br/>≠ 崩溃"]:::todo

    PY --> Z{"Fall ≥ 1<br/>任一发 lying = Y ?"}:::dec
    Z -->|"是"| R["🔴 FALL · 报警<br/>Living → 红 · ? → 红 + 活体未知(非崩溃)"]:::alarm
    R --> T["trigger 锁定 · TODO#1<br/>R=150cm 内 down 无权撤警<br/>只有 cube 连续 2 次阴性可撤"]:::todo

    F -.->|"无资源 / 空返回"| RT["等 X≈60s 重试 · TODO#3<br/>吃满固件 10% 占空 · 长趴持续复查"]:::todo
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
- presence 主判据 = **cube_ff**,z40 兜底(cube_ff 弱/=0/多簇时)。
- `lying(Y/N)` 单独定 fall;`Living_state(Living/?)` 只贴标签。
- **"?" = 仅腿/遮挡测不到,≠ 崩溃。**
- trigger(down)对遮挡/远身体是胡猜;cube 处理中 R=150cm 内 **无权撤警**(只有 cube 连续 2 次阴性可撤)。

## 阈值(已定案,用 case/ 昨日标注数据标定)

- **cube_ff = `0.5`**:≥0.5 用 cube_ff 判 lying;<0.5 转 z40。(躺好信号 0.55-0.92 vs 远/静止 0.00,双峰空档)
- **z40 = `0.4`(现有,不动)**:down 已成立,只判躺(~28)vs 空(~0);站/走上游点云 Z 已排,不用抬。
- 重试间隔 X(≈60s,或 90s 保守)。
- ⚠️ cube 波束宽 → 分不了姿态/家具;姿态=点云 Z,排家具=z40+一次性空房基线。

## 状态

| 已实现(commit) | 内容 |
|---|---|
| c1110ac | cube 目标 = 楼下云 median |
| b1f1adf | z40 dr(0.106)+ XY 逐格 + 堵 cube-free 红漏口 |
| 8982ea6 | 删 far-force(3001 先过滤,守 18s) |
| 7fb173c | 确认后按 down 锁红 30s |

| TODO(未实现) | 内容 |
|---|---|
| #1 | cube 处理中 R=150cm 内 trigger 无权撤警 |
| #2 | cube_ff=0/算不出/多簇 → 用 z40 判读 |
| #3 | 饿死修法:硬 3 发 → 无资源等 X≈60s 重试 |
| 纠正 | presence 优先级:cube_ff 主 / z40 兜底(A+B 弄反了) |
