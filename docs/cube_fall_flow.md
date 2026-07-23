# Cube 跌倒判读流程

AWRL6844 fall pipeline。**cube 是唯一权威**;3001 只负责过滤与起 18s 钟,判决走 cube 的两维返回,trigger 无权在原地否决。

色标:🟢 已实现 · 🟠 TODO(设计定案,未实现) · 🔴 报警 · ⬛ 作废

> 交互版(深/浅色自适应):[`cube_fall_flow.html`](cube_fall_flow.html)

```mermaid
flowchart TD
    A["TI alarm — window-Z / MLP (TLV 321)<br/>—或— 3001 floor_fall(楼下云)"]:::src
    A --> B["down = (w_down AND real_person) OR floor_fall<br/>⚠ down 对遮挡/远身体是胡猜(可信度≈0)"]:::proc
    B --> C{"① tier-1 免费 track_filter<br/>down 持续 ≥ 6s ?(许多 TI alarm 是瞬时误报)"}:::dec
    C -->|"否 <6s"| XF["瞬时误报 · 零成本丢弃<br/>(不算 3001 · 不发 cube)"]:::discard
    C -->|"是"| D["② 起 episode · 18s ExtraMLP 段(tier-2)<br/>3001 ExtraMLP 按需调用 · 平时不调用"]:::done
    D --> E["等 18s<br/>0–18s 只 3001 过滤 · 不发 cube · 不报红"]:::done
    E --> F["③ cube query · tier-3(最贵,最后)<br/>目标 = 楼下云 median bin<br/>门:fresh · 非busy · 速率 · 固件预算"]:::done
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
    R --> T["⭐ 红状态机 = cube 判决(每发定'此刻',≤3发/轮)<br/>升红 = 1发 Y(lying+isPerson) · 撤红 = 连2发 N · 作废(None)不算<br/>Y 令阴性清零;配额尽仍未2N → 红保持<br/>无持久撤警检测 · 无 cloud_up/六关 —— cube 自己的 N 就是撤红"]:::done
    T --> RA["撤红 = 连2N(cube说没了) 或 down 持续清 CUBE_RESET_S(轮结束)<br/>→ episode 复位 + 重新武装下一轮;再倒 = 第2轮<br/>YYY保持 · YNN撤 · YNY阴性清零保持 · Y作废N保持"]:::done

    F -.->|"无资源 / 空返回"| RT["每次跌倒 ≤ 3 次 query(@+18s → +60s → +60s),配额尽停<br/>报警是事件,发出即完成 —— 不无限刷<br/>3×6s=18s/次 ≪ 固件预算(cubeGuard 300/300/3000)· 不 wedge、自终止"]:::done
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

- **成本阶梯(便宜→贵,误报多故先滤):① tier-1 免费 track_filter**(window-Z/MLP `down` 持续 `FALL_PERSIST_S=6s`,<6s 瞬时误报零成本丢弃)**→ ② tier-2 ExtraMLP 按需**(3001 lie-vs-stand,仅 episode 开着时逐帧算,`平时不调用`,空闲走免费几何兜底)**→ ③ tier-3 cube**(最贵,18s 后)。
- **18s** 3001-first:6s 之后再等 18s(tier-2 段),无 cube、不报红。
- 固件 cubeGuard 硬窗口 `300s`(3000 帧 @10fps)。
- 预算 `300` cube-帧/窗口(30s)= **10% 占空**;单发上限 `300` 帧(30s);server 用 60 帧/发。

## 判决原则

- **Fall ≥ 1**:任一发 `lying=Y` 即报。
- presence 主判据 = **cube_ff**(≥0.5),z40 兜底(cube_ff <0.5/=0/多簇 时)。
- `lying(Y/N)` 单独定 fall;`Living_state(Living/?)` 只贴标签。
- **"?" = 仅腿/遮挡测不到,≠ 崩溃。**
- **cube 是权威**:进 cube-query = down 已不可信 → down 不再排/撤 cube;确认后按住红。
- **红状态机(cube 判决,轮次模型)**:升红=1发 Y;撤红=连2发 N;作废(None)不算;Y 令阴性清零;配额(3发)尽仍未2N → 红保持。撤红 = 连2N(cube 说没了)或 down 持续清 CUBE_RESET_S(轮结束→复位重新武装,再倒=第2轮)。**无 cloud_up / 六关走查 / 持久撤警检测**(全删)—— cube 自己的 N 就是撤红,报警是事件、发出即完成。YYY保持 · YNN撤 · YNY阴性清零保持 · Y作废N保持。

## 阈值(已定案,用 case/ 标注数据标定)

- **cube_ff = `0.5`**:≥0.5 用 cube_ff 判 lying;<0.5 转 z40。(躺好信号 0.55-0.92 vs 远/静止 0.00,双峰空档)
- **z40 = `0.4`(现有,不动)**:down 已成立,只判躺(~28)vs 空(~0);站/走上游点云 Z 已排,不用抬。
- **每次跌倒 ≤ `3` 次 query(轮次模型)**:query@+18s → +60s → +60s,配额尽停。cube 的活 = 判这一轮 fall/not-fall(见上"红状态机"),报警是事件、发出即完成,不无限刷。3×6s=18s/次 ≪ 固件预算,不 wedge、自终止。
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
| 0722e | 报警完成模型:每次跌倒 ≤3 query(@+18/+60/+60s)后停;回 60s |
| 0722g | **轮次模型 + 红状态机**:红=cube判决(升红1Y/撤红2N/作废不算/Y清零);删 cloud_up+六关走查+持久撤警;down清=轮结束复位 |

| TODO(未实现) | 内容 |
|---|---|
| #1 | down-gate:只认 down 为真时抓到的 cube(TODO1 3001 veto 重设计的时序补) |
| #2 | 真·多簇 per-cluster cube 分裂仲裁(现由 cube_ff<0.5 近似) |
| — | RANGE_STEP 校正(0.085 vs 实测 10.8cm/bin) |
