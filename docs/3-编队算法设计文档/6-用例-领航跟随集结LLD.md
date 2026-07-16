# 领航跟随集结 LLD

> 对应场景：多机分散位置 → 在统一航线首点集结 → 沿后续航段保持队形

---

## 一、说明

本文档描述领航跟随集结场景（`entity/leader_follower_rally/`）的低层设计，供人阅读，也用于指导代码开发。

本场景在领航跟随保持（`leader_follower_hold/`）基础上新增集结能力，完全遵循《0-HLD.md》架构原则：**不修改现有实体实现和既有单元实现**；允许扩展公共叶类型（`leaf_types.py`）、Context（`context.py`）及实体边界类型（`EntityInputS`/`EntityInitS`/`EntityOutputS`）。新建实体放在新目录，复用/扩展所需的单元族。

**关于实体代码复用**：`RallyLeaderEntity` 与现有 `LeaderEntity` 有大量结构相似的代码（位置解算、跟踪、输出回填等）。当前版本选择**直接新建完整实体**，原因是：① Hold 场景不受影响，无需回归测试；② FormationTask（Hold vs Rally）是二选一，无法在同一实例中兼容；③ 功能验证优先，过早提取基类会增加当前实现风险。待集结功能稳定、所有场景装配方案确定后，可一次性提取 `LeaderEntityBase` / `FollowerEntityBase` 消除重复，改动集中、风险可控。

---

## 二、总体策略与状态机

### 2.1 设计原则

**集结期间三机平等**：JOINING 阶段（cmd.step=0）不区分长机/僚机，所有飞机用同一套 `RallyJoinPos` 算法飞向各自的预设松散目标点 M_i。只有当全部飞机完成集结（均进入 EXITED 状态）后，才切换到 CATCHUP 阶段，此时 R01 开始作为编队参考源生成槽位目标，R02/R03 进入跟随模式。

**CATCHUP 阶段过渡**：全员切出时各机在沿航迹方向的位置是分散的——最晚到达 M_i 的飞机直接切出后已向前飞出一段距离，而早到的飞机（包括掌机）刚刚从盘旋圆切出，位置可能落后。CATCHUP 阶段与 LOOSE 使用**完全相同**的位置解算（`SlotGeometry`，直接给出真实槽位位置与槽位自身速度前馈），沿航迹的加减速收敛交给下层控制律（`PidCompose` 前向 PPI 外环按真实位置误差生成速度修正）完成；CATCHUP 只是 Rally 任务状态机里独立的一个阶段门控（`_all_catchup_ok`，位置+航向双阈值），用于确认三机间距已收敛到松散队形要求后再进入 LOOSE。

**长机角色切换时机**：CATCHUP 开始后掌机沿统一 `route` 飞行，僚机以掌机当前位置为参考动态计算各自槽位目标。LOOSE/COMPRESS/HOLD 阶段行为与领航跟随保持场景完全相同。

**单向生命周期裁决**：正式控制器只允许一次 `STANDBY → RALLY → HOLD`。运行中的
`RALLY/HOLD → STANDBY/NONE` 不属于取消或中断协议，`NONE/STANDBY → RALLY` 也不构成受支持的二次集结。
需要开始新的集结生命周期时，控制器必须先对长机和全部僚机实体执行显式 `reset()`，由该操作同时清除
Rally 任务固定计划、僚机入站锁存计划和 `RallyJoinPos` 位置计划，再从 STANDBY 开始。

### 2.2 四阶段集结策略

---

#### 第一步：集结航线与松散目标点（初始化时自动计算）

**当前有效航线定义集结和任务飞行**：

控制器初始从 `route_file` 构造有效航线；用户采用避障规划结果后，`leader_route_override` 替换配置航线并触发实体重新初始化。以下 `route` 均指实体本次初始化收到的当前有效航线，不固定等同于原始配置文件内容。

- **A = `route[0]`**：集结中心，掌机在 JOINING 阶段的目标位置，也是松散队形中心。
- **A1 = `route[1]`**：首段终点，A→A1 同时确定集结队形朝向和集结完成后的出航方向。
- **`route[2...]`**：集结完成后继续执行的任务航点。

```text
A  = route[0]  集结中心
A1 = route[1]  第一航段终点（用于推导集结和出航方向）

ψ_h = atan2(A1_north − A_north, A1_east − A_east)  # 第一航段水平航向，初始化时自动推导

Δeast  = looseScale × (slot.x × cos ψ_h + slot.z × sin ψ_h)
Δnorth = looseScale × (slot.x × sin ψ_h − slot.z × cos ψ_h)

掌机松散目标：M_leader = A（飞向集结区起点）
僚机松散目标：M_i.east/north = A.east/north + [Δeast, Δnorth]
             高度：A.h + slot.y（固定差，不随 looseScale 扩展）
```

各机的松散目标点由仿真层在配置加载时自动计算，**无需在配置文件中逐机写死**。这里是一个有意保留的
**ENU 水平集结平面适配**：`M_i` 用于构造水平盘旋圆，因此把 FUR 槽位实例化在与 A→A1 水平航向
对齐、倾角为零的合成平飞 FUR 中；`slot.x/z` 投影到 ENU 水平面，`slot.y` 成为天向高度差。即使
第一航段爬升，也不把真实航迹倾角耦合进 `M_i`。它不是随当前航迹旋转的三维 FUR 实时槽位；进入
CATCHUP/LOOSE/COMPRESS 后，实时槽位由 `SlotGeometry` 按长机三维 FUR（前、上、右）处理。
当 `ψ_h=0`（正东）时，本适配退化为 east = slot.x，north = −slot.z（z_right = 南向）。

三机示例（A=(0, 5000)，ψ_h=0°（正东），looseScale=3）：

| 节点       | 队形槽位 (x_m, z_m) | slot→ENU (east, north) | × looseScale   | M_i (east, north) |
| ---------- | ------------------- | ---------------------- | -------------- | ----------------- |
| R01（长机）| (0, 0)              | (0, 0)                 | (0, 0)         | (0, 5000) = A     |
| R02        | (-54, -58)          | (-54, +58)             | (-162, +174)   | (-162, 5174)      |
| R03        | (-54, +58)          | (-54, -58)             | (-162, -174)   | (-162, 4826)      |

---

#### 第二步：JOINING 阶段（切线进圆 + 盘旋协调汇合）

每架飞机独立运行 `RallyJoinPos` 单元。该单元对外有 `STANDBY → FLYING → LOITERING → EXITED` 四个状态，其中 `FLYING` 内部再细分为待命圆切出和公切线直飞两个私有子阶段，不增加任务或通信枚举。

**盘旋圆几何（init 时按任务航向定死，不随到达方向变化）**：

```python
C = M_i + R × (−sin θ_m, cos θ_m)   # 圆心，θ_m = mission_heading_rad，R 向任务航向左侧偏置
θ_slot = atan2(M_i.north − C.north, M_i.east − C.east)  # M_i 在圆上的固定角度
```

这样摆放保证 M_i 处的 CCW 切线方向恒等于任务航向 θ_m，与飞机从哪个方向飞来无关——这是本节相对旧版最核心的改动：旧版用"到达 M_i 时的速度方向"摆圆心，导致盘旋圆朝向和实际到达方向绑定，一旦到达方向和任务航向差异较大（例如飞机从任务航线下游一侧飞向集结点），切出瞬间指令会相对盘旋指令发生大角度跳变，表现为"切出后先反向飞一段再掉头"。改为按任务航向定死圆心后，不管飞机从哪个方向飞来，只要沿圆弧飞到 M_i 就必然对齐任务航向，跳变问题不再出现。

**STANDBY**：进入待命的第一拍，根据本机当前位置和航向反推等半径 CCW 待命圆，随后持续输出圆上投影目标、切向速度和曲率前馈。开始集结前，长机与僚机都在该状态独立盘旋。

**FLYING**：开始集结时每架飞机按自己的待命圆和集结圆一次性规划转移路径，对外始终广播 `FLYING`，内部顺序如下：

1. 调用 `arc_path.common_tangent()` 求两个等半径 CCW 圆的定向外公切线，并锁存待命圆切出点 `T_local`、集结圆切入点 `T_rally`、对应圆周角和直线长度；规划完成后不随实时位置重算。
2. `ARC_TO_TANGENT`：继续沿待命圆 CCW 飞行。到 `T_local` 的剩余角不大于 `10°` 时，同一拍切换到直线段；若离散步进跨过切点，也根据上一拍剩余角检测到跨越并切换，避免多绕一圈。
3. `LINE_TO_RALLY_ENTRY`：直飞锁存的 `T_rally`，满足现有切入捕获半径后转入 `LOITERING`。

待命圆和集结圆圆心距离不大于 `0.5m` 时直接进入 `LOITERING`。公切线无解时，退化为开始集结那一拍的当前位置到集结圆 CCW 切线并锁存；若点到圆切线仍无解，再退化为直飞 `M_i`。

开始 JOINING 时，`RallyJoinPos` 一次性锁存基础水平航程 `L_i`：待命圆当前位置到本地切出点的 CCW 弧长、公切线（或实际退化路线）长度，以及集结圆切入点到 `M_i` 的 CCW 弧长之和。未获得计划前仍按名义速度飞行；计划有效后，待命圆弧、公切线和集结圆弧均使用同一套全航程协调速度。

- **切入触发半径**：`d_3d < min(arrival_radius_m, _arc_capture_radius_m)` 才转入 LOITERING，其中
  `_arc_capture_radius_m = R × tan(ψ_max)`（`ψ_max = _MAX_ARC_CAPTURE_HEADING_JUMP_RAD ≈ 5°`）按**当前
  `loiter_radius_m` 反解**得到，不再是与半径无关的固定常量。原因：T 是圆上固定点，FLYING 阶段全程直飞
  T（航向恒定=T 处切向），但 LOITERING 第一拍的航向按飞机*此刻实际角度*算切向——触发半径 d 与盘旋半径
  R、跳变角 ψ 满足 `ψ = atan(d/R)`：半径越小，同样的固定 d 换算出的跳变角越大（实测 R=200m 时 4.26°，
  R=50m 时 16.64°，R=20m 时 36.14°，R=10m 时 56.12°，均用旧版固定 15m 触发半径复现）。改成按 R 反解 d
  后，跳变角在合法半径范围内恒被压在 ψ_max 量级，不再随 R 变小失控放大。
- **切入/切出几何的配置期校验**（`validate_capture_geometry()`，`rally_join_pos.py` 模块函数，`RallyJoinPos.init()`
  与 `_ConfigLoader.validate()` 共用同一份逻辑，避免"validate 通过但 init 失败"）：
  - `worst_case_speed_mps = max(approach_speed_mps, loiter_speed_min_mps)`（FLYING 直飞用 approach 速度，
    LOITERING 圆弧巡航最慢也有 `loiter_speed_min_mps`，两者中更快的那个决定单步走过的距离更长，是更保守的边界）
  - `required_capture_radius_m = max(_MIN_ARC_CAPTURE_RADIUS_M, _MIN_ARC_CAPTURE_STEP_MARGIN × worst_case_speed_mps × control_period_s)`
    （触发半径必须显著大于单个控制周期内飞机能走过的距离，否则离散步进可能直接跨过窄触发窗口、错过切入判定）
  - `loiter_radius_m` 必须 `≥ required_capture_radius_m / tan(ψ_max)`，否则拒绝——半径太小时按上面的公式
    反解出的 d 本身就小于单步安全边界，无法同时满足"跳变角小"与"步进不越窗"两个约束
  - `arrival_radius_m` 必须 `≥ required_capture_radius_m`——`min(arrival_radius_m, _arc_capture_radius_m)`
    里 `arrival_radius_m` 同样可能是生效的那个更小值，只校验 `loiter_radius_m` 而不管 `arrival_radius_m`
    配置得过小（如 1m）会让这条 min 悄悄绕过上面的半径校验，实际触发窗口还是窄于单步安全边界
- **退化边界**：圆心近零已显式处理为直接进入 `LOITERING`；仅在公切线和当前点到集结圆切线都无解时，才退化为直飞 `M_i`。
- **`loiter_speed_min/max_mps` 的推导与校验**（`loiter_speed_bounds()`，`leader_follower_rally/__init__.py`
  模块函数，`RallyFollowerEntity.init()`/`RallyLeaderEntity.init()`/`_ConfigLoader.validate()` 三处共用同一份逻辑）：
  - `loiter_min`/`loiter_max` 分别从 `velCmdLimit.forwardMin`/`forwardMax` 取值，未配置或非正值时各自独立退回
    默认兜底 14/25 m/s——两者是**独立**回退的，只显式配置其中一侧时，另一侧会退到默认值，可能与显式配置的
    值反序（如只配 `forwardMax=10` → `(14, 10)`，只配 `forwardMin=30` → `(30, 25)`），必须在这里就地校验
    `loiter_max > loiter_min`，否则该配置能通过 `_ConfigLoader.validate()`，直到实体真正构造时才在
    `RallyJoinPos.init()` 报 `ERR_MODULE_INIT_FAILED: loiter speed limits invalid`，报错时机被推迟、原因也不直观

**LOITERING**：

- 到达切入点 T 附近后，顺势沿盘旋圆做 CCW 圆周飞行（圆心/M_i 处切线方向已在 init 时定死，不再重新摆放）
- 控制指令：`selfCmd.pos = 期望半径圆上、飞机当前角度处的投影点`（**不是圆心**），`selfCmd.v = 切线速度`，
  `dVPsi = v / loiter_radius_m`（CCW 向心前馈，用**期望半径**而非飞机实时半径）
  > 目标点如果取圆心，位置误差（侧偏）会恒等于飞机此刻的实际半径，跟期望半径无关——控制律没有把半径拉回
  > `loiter_radius_m` 的趋势。实测这样会导致实际盘旋半径在很宽范围内漂移（同一场景下 23~222m 都出现过，
  > 期望是 200m）。改成"圆上投影点 + 期望半径前馈"后，侧偏才是真正意义上的"半径误差"，能收敛到期望值
  > （实测收敛到约 200~202m）。这个思路和 `RouteInterp` 处理避障圆弧航段（`arc_path.project_arc`：
  > 投影到弧上、目标点=投影点）是同一套模式，只是盘旋圆没有起止角、不需要夹在弧段范围内。
- **经过 M_i 的检测**：基于角度，不依赖实际轨道半径：
  - `arc_angle = (θ_slot − θ_self) mod 2π`（CCW 剩余弧角，0~2π）
  - `ang_dist = min(arc_angle, 2π − arc_angle)`（到 M_i 的最短弧角——**对称**，分不清"快到"和"刚过"）
  - `ang_dist > _SLOT_ANG_AWAY (≈60°)` → 标记"已远离"；相邻拍有向剩余角从点前小角回绕到点后大角，
    且对应 CCW 前进量不超过 `π` → 确认真实跨零，不要求前一拍先命中更窄的 20° 近窗
  - 小于 `_EPSILON_HORIZ`（0.5m）弧长的跨零候选按位置抖动处理，保留点前样本等待后续真实推进；
    点后近窗切入和反向角度跳变不满足有向 CCW 关系，不消费圈数
  - 刚从切入点 T 进弧时，只有 **T 到 M_i 的真实 CCW 弧长本身位于点前 60° 范围**才预置"已远离"，
    允许首次真实路过 M_i 就评估切出；弧长不小则必须先飞过远离窗口。
    > `ang_dist` 是对称弧距，分不清"T 恰好在 M_i 之前一点"（真快到了，弧长小）和"T 恰好在 M_i 之后
    > 一点"（弦长虽近，CCW 方向其实还要绕近一整圈才能到）。如果进弧就无条件置位"已远离"（不看真实弧长），
    > 后一种情况会被误判成"已到达"，在没有真正沿圆弧飞完的情况下就直接切出——原样复现了这次重构本想
    > 根除的"切出瞬间指令跳变"问题。用真实弧长而非对称弧距来决定初始"已远离"状态即可避免。
- 每次经过 `M_i` 时按已锁存的 `remaining_loops` 处理：大于零时减一并继续盘旋；等于零时立即切出并进入 `EXITED`。`loop_counts[i]` 是基础路径首次到达 `M_i` 后需追加的完整圈数，不包含切入点到 `M_i` 的基础圆弧。

**EXITED**：从 M_i 位置沿任务航向（`mission_heading_rad`）直飞，交由 CATCHUP 接管；切出瞬间的指令方向恒等于任务航向，不再依赖飞机是"先到"还是"最后到"。

> **一致性**：切入点 T 到 M_i 的首次路过，与盘旋过程中每次经过 M_i，使用同一套角度检测。圈数在每次真实跨点时恰好消费一次，零圈即切出；切出不再受当前时刻相对 `T_ref` 的门控影响。

**固定协调计划**（长机 Rally 任务，只生成一次）：

每架飞机上报有限且非负的基础航程 `L_i`。共同圆周为 `C = 2*pi*R`，增加 `n` 个完整圈后的航程和相对可达区间为：

```text
D_i(n) = L_i + n * 2*pi*R
I_i(n) = [D_i(n) / V_max, D_i(n) / V_min], n >= 0
```

长机收齐自身及全部期望僚机的 `L_i` 后，令公共距离 `X = V_max*T`、
`r = V_min/V_max`。最早可行解必有某架飞机的 `D_i(n)=X`；对每个节点的模圆周相位只枚举一次，计算其他
航程序列在 `X` 之前最近点的最大相位差 `G`，再直接跳到满足 `G <= (1-r)X` 的首个同相位距离。
候选数严格等于节点数，不随所需圈数增长。基础区间直接相交时显式返回全员零圈；有限浮点范围内无可表示
候选时明确失败。搜索同时得到 `loop_counts[node_id] = n_i`，并锁存：

```text
T_ref = plan_start_s + earliest_common(I_1(n_1), ..., I_N(n_N))
```

`plan_start_s` 取长机首次收齐全队有效 `L_i` 并生成计划时的当前仿真时刻。`T_ref`、`loop_counts` 和 `t_ref_valid=True` 在本次集结生命周期内固定，直到实体 `reset()`；不重新搜索、不后移、不按已切出飞机改变参与集合。未收齐有效 `L_i` 时计划无效，各机沿已规划路径以名义速度继续飞行，期间经过 `M_i` 不消费后续计划分配的圈数。`reachedSlotOnce` 仅为诊断量，不参与计划计算。

计划通过 `formation.leader` 广播重复下发 `{t_ref, t_ref_valid, loop_counts}`。僚机只读取本机 ID 的圈数并在首次有效计划时锁存；重复收到同一计划不得累加。有效计划后，`T_ref` 只用于全航程调速，`loop_counts` 决定每次经过 `M_i` 时是否切出。

**通信链路**（见图 4）：

- 僚机 → 长机：`formation.follower_status` 消息，含 `{pos, planned_path_length_m, rally_state, reached_slot_once, arrived}`
- 长机 → 僚机：`formation.leader` 消息，含 `{cmd, slot_scale, t_ref, t_ref_valid, loop_counts, leader_state}`；其中 `cmd["leader"]` 携带长机有效运动指令，供僚机槽位坐标系建系

**JOINING → CATCHUP 门控**（`_all_participants_exited`）：

1. 长机自身 `rally_state == EXITED`
2. 所有 `expectedFollowerIds` 机均 `rally_state == EXITED`
3. 以上状态均在 `stale_timeout_s` 内有效

> **说明**：各机 EXITED 的时间先后由基础航程和分配圈数共同决定。正常可达时，最后一次经过 `M_i` 自然发生在 `T_ref` 附近；离散步进只影响数值时刻，不改变按圈切出的语义。全员切出瞬间沿航迹位置仍可能分散，后续由 CATCHUP 收敛相对间距。

#### 已知限制与待办

以下是 JOINING/`RallyJoinPos` 当前设计已确认的取舍或遗留缺口，记录以便后续排期，均不是本次改动引入的新缺陷：

1. **两级切线都无解时仍会直飞 M_i**（见"FLYING"小节）：公切线失败后会先尝试开始集结时的当前点到集结圆切线；只有当前点也位于集结圆内或圆上、该切线仍无解时才退化为直飞 M_i。待命圆与集结圆圆心近零不走此兜底，而是直接进入 `LOITERING`。
2. **进场角度不利的飞机可能要多飞近一整圈基础圆弧**：切入点 T 到 M_i 的真实 CCW 弧长完全由飞机相对盘旋圆的进场方向决定；如果这个弧长恰好接近 360°，飞机仍须先飞完这段弧才能以正确航向到达 M_i 切出——这是保证"切出航向恒等于任务航向"这个几何约束的必然代价，不是能单靠调参消除的问题。极端情况下会让单机集结耗时明显变长；如果后续场景对集结总时长敏感，需要评估是否要在集结点/进场方向的选取上做额外约束来规避大弧长进场。
3. **`loiter_radius_m` 有一个由 `approach_speed_mps`/`control_period_s` 反推出的隐式下限**（见 `RallyJoinPos.init()`）：半径太小时，切入圆弧的触发半径会被地板值或离散步进距离压过 5° 航向跳变角上限，init 会直接拒绝。这个下限不是配置里能直接看到的一个数字，而是每次 init 时按当前 `approach_speed_mps`/`control_period_s` 现算的，调这两个参数时要留意联动影响。

---

#### 第三步：CATCHUP 阶段

**触发条件**：全员 EXITED，Rally 任务从 step=0 切换到 step=1。

**目标**：各机与掌机之间的相对间距（含沿航迹方向的散布）收敛到松散队形要求。

##### 位置解算：与 LOOSE 完全相同

CATCHUP 不再有专属的位置解算算法，直接复用 `SlotGeometry`（与 LOOSE/COMPRESS/HOLD 同一套）：

```text
slot = 掌机当前位置 + scale × rotate(编队偏置, 掌机航迹)   # scale = looseScale
selfCmd.pos = slot
selfCmd.v   = 槽位自身速度前馈（随掌机运动 + 队形刚体旋转前馈）
```

沿航迹方向的加减速（落后加速、超前减速、到达后跟随槽位自身速度）完全由下层控制律 `PidCompose` 的前向通道完成：外环按真实位置误差生成速度修正，内环把速度误差转成加速度，`forwardMin/forwardMax` 限幅避免倒飞。这与领航跟随保持场景（`leader_follower_hold/`）里僚机的前向通道是同一套逻辑，CATCHUP 与 LOOSE 唯一的区别只是 `slotScale`（此时仍是 `looseScale`）和 Rally 任务状态机所处的阶段。

> 历史设计（已废弃）：曾经引入过 `CatchupAlign` 单元，用"杆模型"把飞机投影到过 M_i 点、平行任务航向的直线上，人为将前向位置误差钉为 0，再单独用速度调制追赶——这是因为当时 `PidCompose` 按**本机自身航迹系**投影位置误差，目标落在机尾方向时可能形成"越滚越偏"的横侧向正反馈。后来 `PidCompose` 改为按**目标（selfCmd）自身航迹系**投影位置误差（见 [pid_compose.py:110-121](../../src/algorithm/units/algo/pos_track/pid_compose.py#L110-L121)），横侧向切入由控制律统一处理；前向通道仍按 `vel_cmd = vel_ff + kpPos × posErr` 生成追赶速度，是否允许负速度指令由 `forwardMin` 限幅决定，本场景配置为正值以禁止倒飞。`CatchupAlign` 的投影/锁航向/速度调制因此成为与下层控制律重复的多余逻辑，已删除。

##### CATCHUP → LOOSE 门控

```text
posErr_m = dist3d(selfState, selfCmd)   # 与 LOOSE 阶段同一套 FollowerBroadcast 广播口径
```

Rally 任务检查：所有期望僚机同时满足 **`posErr_m < catchup_radius_m`（默认 200 m）** 和 **航向误差 `< 0.17 rad`**，并连续保持 3 秒后切换到 LOOSE。航向阈值和连续保持时间是当前实现的内部固定值，不从场景配置读取。

---

#### 第四步：LOOSE → COMPRESS → HOLD（松散收紧）

长机沿统一 `route` 飞，僚机跟随 `SlotGeometry` 槽位。

| 子阶段   | cmd.step | 说明                                                      |
| -------- | -------- | --------------------------------------------------------- |
| LOOSE    | 2        | 松散间距跟随，等待收敛（误差 < ε_loose 持续 T_stable_s）  |
| COMPRESS | 3        | scale 线性从 looseScale → 1.0，持续 compressTime_s        |
| HOLD     | —        | scale=1.0，输出 FormationAnalysisS                        |

---

### 2.3 子阶段编码

`cmd.stage` 在 JOINING/CATCHUP/LOOSE/COMPRESS 全程保持 `RALLY`，完成后切 `HOLD`：

| `cmd.step` | 子阶段   | 含义                                           |
| ---------- | -------- | ---------------------------------------------- |
| 0          | JOINING  | 三机平等飞向 M_i，盘旋协调，等待全部 EXITED    |
| 1          | CATCHUP  | 沿任务航向直飞，速度调制收敛到松散队形相对间距 |
| 2          | LOOSE    | 松散间距三维槽位跟随，等待收敛                 |
| 3          | COMPRESS | 线性压缩至最终间距                             |

### 2.4 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `loiter_radius_m` | 200 m | 盘旋圆半径 |
| `arrival_radius_m` | 100 m | 触发到达判断的距离阈值 |
| `mission_heading_rad` | — | 切出后飞行方向，由 A→A1（集结航线第一航段方向）自动推导，不在配置中写死 |
| `loiter_speed_min/max` | 14/25 m/s | 盘旋速度上下限（固定翼约束） |
| `catchup_radius_m` | 200 m | CATCHUP→LOOSE 门控阈值（三维槽位距离，米） |
| `loop_counts[node_id]` | 计划生成时确定 | 本机首次到达 `M_i` 后需要追加的完整圈数 |

切出不使用剩余时间阈值或独立的到达时间参数。固定计划生效后，飞机每次真实跨越 `M_i` 时仅检查本机 `remaining_loops`：大于零则精确减一并保持 `LOITERING`，等于零则在该次跨越进入 `EXITED`。`T_ref` 仅用于全航程协调速度计算，不参与切出门控。

---

## 三、新增叶类型

以下类型新增到 `src/algorithm/context/leaf_types.py`。

### 3.1 `RallySlotScaleS` — 槽位缩放因子

```python
@dataclass
class RallySlotScaleS:
    """集结阶段的槽位偏置缩放因子。scale=1.0 为最终队形，>1.0 为松散放大。
    注意：需要跨拍保留，且被 FormationTask/Rally 写、PosCalc/SlotGeometry 读，故进 Context。"""
    scale: float = 1.0
    scaleRate: float = 0.0   # scale 对时间的导数（1/s）；LOOSE 为 0，COMPRESS 为负值
    # SlotGeometry 用 scaleRate 计算因压缩产生的额外速度前馈，避免在单元内存储上一拍 scale
```

### 3.2 `FollowerStateS` — 僚机集结状态快照

```python
@dataclass
class FollowerStateS:
    """单架僚机向长机回报的集结状态。注意：id 与节点 ID 对应；posErr 为该机到当前目标的合距离。"""
    id: str = ""
    pos: PosInEarthS = field(default_factory=PosInEarthS)  # 实际位置
    posErr_m: float = 0.0        # 到当前目标（M_i 或松散槽位）的合距离，米
    headingErr_rad: float = 0.0  # 当前航向与目标航向之差的绝对值，弧度
    arrived: int = 0             # 兼容旧协议；新协议以 rally_state == EXITED 为准
    valid: bool = False          # 本帧数据是否有效（收到最新报文则置 True）
    lastUpdate_s: float = 0.0    # 最近一次收到该机报文的仿真时间戳，秒
    plannedPathLength_m: float = -1.0  # 开始 JOINING 后锁存的基础水平航程 L_i；非法哨兵值表示未规划
    rally_state: str = "FLYING"  # 集结汇合状态：FLYING / LOITERING / EXITED
    reachedSlotOnce: bool = False  # 是否已至少一次路过 M_i，汇合过程诊断量
```

### 3.3 `FormationAnalysisS` — 编队分析快照

```python
@dataclass
class FormationAnalysisS:
    """集结完成后的一次性编队质量分析。注意：仅作边界诊断量输出，不进 Context。"""
    posErrMax_m: float = 0.0    # 期望僚机中的最大位置偏差，米（仅统计 expectedFollowerIds 里有效节点）
    posErrRms_m: float = 0.0    # 期望僚机位置误差 RMSE，米
    inPositionCount: int = 0    # 期望僚机中满足精度要求的机数
    totalCount: int = 0         # 期望参与集结的总机数（= len(expectedFollowerIds)，不受断链影响）
```

同时在 `leaf_types.py` 补充对应的 `copy_*` 函数：`copy_rally_slot_scale`、`copy_follower_state`、`copy_formation_analysis`。

实体代码中出现的三个辅助函数均来自现有 `context.py` / `leaf_types.py`（如不存在则在 `context.py` 补充）：

| 函数 | 语义 | 来源 |
| --- | --- | --- |
| `copy_position(src, dst)` | 逐字段复制 `PosInEarthS`（east/north/h），避免对象别名 | 参考现有 `copy_motion` 的实现风格 |
| `zero_velocity(v)` | 将 `VdInEarthS` 所有数值字段原地清零 | 新增，与 `copy_motion` 同文件 |
| `zero_acceleration(a)` | 将 `AccInEarthS` 所有数值字段原地清零 | 新增，与 `copy_motion` 同文件 |

---

## 四、Context 扩展

在 `FormContextS` 中新增集结专用字段（进 Context 的条件：跨拍保留 + 多单元读写）：

```python
@dataclass
class FormContextS:
    # ... 已有字段 ...
    slotScale: RallySlotScaleS = field(default_factory=RallySlotScaleS)
    # 被 FormationTask/Rally(写) 与 PosCalc/SlotGeometry(读)

    followerStates: list[FollowerStateS] = field(default_factory=list)
    # 被 Inbound/FollowerStatus(写) 与 FormationTask/Rally(读)
    # 注意：list 在移植 C 时改为定长数组+计数器

    rally_t_ref: float = 0.0
    rally_t_ref_valid: bool = False
    rally_loop_counts: dict[str, int] = field(default_factory=dict)
    # 被 RallyLeaderFollower(写) 与 RallyJoinPos(读)；同一固定计划整体更新
```

`reset_context` 同步扩展：`slotScale.scale = 1.0, slotScale.scaleRate = 0.0`，`followerStates.clear()`，并清空固定计划字段。

---

## 五、新增流程组单元

### 5.1 FormationTask/Rally — 集结任务编排

**文件**：`units/process/formation_task/rally.py`

作用：管理 `JOINING→CATCHUP→LOOSE→COMPRESS→HOLD` 状态机（`cmd.step` 编码为 `RallyPhaseE`
`JOINING=0/CATCHUP=1/LOOSE=2/COMPRESS=3`），写出 `cmd`（stage/step/pattern）、`slotScale` 与一次性锁存的
固定计划（`t_ref`/`t_ref_valid`/`loop_counts`），供 JOINING 阶段的 `RallyJoinPos` 全航程调速和按圈切出。

> 本节原描述"APPROACH→LOOSE→COMPRESS"三段式（`arriveHold_s`/`_arrive_timer`/`all_followers_arrived()`
> 按僚机锁存的 `arrived` 标志判定到达），是 `RallyJoinPos`（切线进圆汇合）之前的旧设计。当前实现在
> APPROACH 与 LOOSE 之间插入了独立的 CATCHUP 子阶段（`_all_catchup_ok()` 按位置+航向双阈值门控），且
> JOINING 阶段的到达判定完全交给 `RallyJoinPos.state`/`reached_slot_once`（见第二步 JOINING 阶段说明），
> `arriveHold_s`/`_arrive_timer` 已不存在。以下按实际实现更正。

#### 5.1.1 抽象类扩展

`FormationTaskInitS` 与 `FormationTaskInputS`/`FormationTaskOutputS` 基类不变；`Rally` 子类扩展输入/输出端口结构体：

```python
@dataclass
class RallyTaskInitS(FormationTaskInitS):
    looseScale: float = 3.0               # 松散槽位放大倍数（松散间距=最终间距×looseScale）
    convergenceRadius_m: float = 5.0      # LOOSE→COMPRESS 槽位误差阈值，米
    stableHold_s: float = 5.0             # LOOSE→COMPRESS 需稳定的时间
    compressTime_s: float = 30.0          # COMPRESS 阶段持续时间（scale 从 looseScale→1.0）
    tightRadius_m: float = 2.0            # COMPRESS→HOLD 精度阈值，米
    expectedFollowerIds: list[str] = field(default_factory=list)
    # 期望参与集结的僚机 ID 列表；空列表→各门控立即通过（测试用）
    staleTimeout_s: float = 2.0           # 超过此时长未收到某机报文则视为数据失效
    targetPattern: int = 0
    # 集结只用单队形（formPos 第 0 行），cmd.pattern 恒写入此索引，供 SlotGeometry 查槽位
    dt_s: float = 0.02                    # 控制周期（秒）；进 InitS 才能在 init 时校验 > 0
    # 以下为 RallyJoinPos 参数，Rally 任务只透传给实体，不参与本单元自身状态机
    loiter_radius_m: float = 200.0        # 盘旋圆半径，米
    arrival_radius_m: float = 100.0       # 进入盘旋的触发距离，米
    catchup_radius_m: float = 200.0       # CATCHUP→LOOSE 位置误差阈值（dist3d to slot），米
    catchup_heading_thresh_rad: float = 0.17  # CATCHUP→LOOSE 航向误差阈值，弧度（≈10°）
    catchup_stable_s: float = 3.0         # CATCHUP→LOOSE 需连续满足的时长，秒

@dataclass
class RallyTaskInputS(FormationTaskInputS):
    # 继承 remote: RemoteCmdS, cmd: FormSnapshotS
    followerStates: list[FollowerStateS] = None  # 端口 → Context.followerStates
    now_s: float = 0.0    # 当前仿真时间（秒），由实体从边界输入注入，用于超时判断
    leader_path_length_m: float = -1.0      # 长机自身锁存的基础水平航程 L_i
    leader_join_exited: bool = False        # 长机自身是否已 EXITED

@dataclass
class RallyTaskOutputS(FormationTaskOutputS):
    # 继承 cmd: FormSnapshotS
    slotScale: RallySlotScaleS = None       # 端口 → Context.slotScale
    rallyCompleted: bool = False            # COMPRESS→HOLD 正常完成时置 True，仅该拍有效；实体据此输出 FormationAnalysisS
    t_ref: float = 0.0                      # 全队整数圈可达区间的最早公共时刻
    t_ref_valid: bool = False               # 是否已收齐长机与全部期望僚机的有效基础航程
    loopCounts: dict[str, int] = field(default_factory=dict)  # 每机额外完整圈数
```

#### 5.1.2 Rally 子类实现逻辑

**`init`**：存储配置参数，初始化内部计时器 `_catchup_stable_timer`、`_stable_timer`、`_compress_elapsed`、
单向生命周期锁存 `_rally_started` 及固定计划锁存 `_t_ref/_loop_counts/_plan_ready`。参数合法性断言（违反则抛 `ValueError`）：`looseScale >= 1.0`、`compressTime_s > 0`、
`staleTimeout_s > 0`、`dt_s > 0`、`loiter_radius_m > 0`、`0 < loiter_speed_min_mps < loiter_speed_max_mps`。

**`step`** 顶层逻辑（先处理 remote，再按 `cmd.step` 路由）：

> **每拍开头先置 `y.rallyCompleted = False`，并调用 `_write_plan(y)` 复制固定计划锁存**，再进入
> remote/step 路由。`t_ref/t_ref_valid/loopCounts` 在显式 `reset()` 前保持固定，不能按帧清成无效。

```text
remote == NONE:
  # 低层兼容停控命令；固定计划与 _rally_started 均保持锁存
  若 cmd.stage in {RALLY, HOLD}:
    只清阶段计时器，固定协调计划继续锁存
  输出 cmd.stage=NONE, cmd.step=JOINING(0), cmd.pattern=0, slotScale.scale=looseScale, scaleRate=0
  return

若 _rally_started 且 cmd.stage == NONE:
  # 本次任务已进入过 RALLY；未经显式 reset 的 RALLY/STANDBY 重启均拒绝
  保持 cmd.stage=NONE 与兼容停控输出
  return

若 _rally_started 且 remote == STANDBY:
  # 反向 STANDBY 不改变已开始任务的阶段
  将本拍按 remote=RALLY 继续当前 RALLY/HOLD 路由

remote == HOLD:
  # 正式流程只允许 Rally 自身完成后进入 HOLD；该分支不提供运行中取消语义
  若 cmd.stage == RALLY:
    只清阶段计时器，固定协调计划继续锁存
  输出 cmd.stage=HOLD, cmd.step=JOINING(0), cmd.pattern=targetPattern, slotScale.scale=1.0, scaleRate=0
  return

remote == RALLY:
  若 cmd.stage == HOLD:                # 当前实体生命周期已完成；忽略 RALLY 重启
    输出 cmd.stage=HOLD, cmd.step=JOINING(0), cmd.pattern=targetPattern,
         slotScale.scale=1.0, slotScale.scaleRate=0
    return                             # 新生命周期必须先显式 entity.reset()
  若 cmd.stage in {NONE, STANDBY}:     # 新实体生命周期首次进入集结
    reset 所有计时器
    cmd.step = JOINING(0)
  锁存 _rally_started=True
  # cmd.stage == RALLY → 继续集结，按 cmd.step 路由
```

**`step`** 按 `u.cmd.step` 路由（只在 `remote.stage == RALLY` 且 `cmd.stage != HOLD` 时执行）：

```text
辅助函数 is_valid(entry):
  entry 未找到 OR valid==False OR (now_s - lastUpdate_s) > staleTimeout_s → False；否则 True

辅助函数 all_participants_exited(leader_exited):
  （用于 JOINING→CATCHUP：期望僚机与长机自身是否都已 RallyJoinPos.state==EXITED）
  长机未 EXITED → False；expectedFollowerIds 为空 → True（长机已 EXITED 即可）
  对每个 id：entry 缺失 → False；已 EXITED → 视为终态跳过（不因随后丢链被撤销）；
             否则要求 is_valid(entry)，新鲜报文但 state!=EXITED → False

辅助函数 all_catchup_ok():
  （用于 CATCHUP→LOOSE：三维位置 dist3d(self,slot) 和航向误差双阈值）
  expectedFollowerIds 为空 → True
  对每个 id：entry 缺失/无效 OR posErr_m>=catchup_radius_m OR headingErr_rad>=catchup_heading_thresh_rad → False

辅助函数 all_followers_ok(threshold_m):
  （用于 LOOSE→COMPRESS 和 COMPRESS→HOLD：检查槽位误差收敛）
  expectedFollowerIds 为空 → True
  对每个 id: is_valid(entry)==False OR posErr_m >= threshold_m → False
  全部通过 → True

（说明：各子阶段先计算 next_step/next_stage，本拍统一输出新值，避免输出与内部状态矛盾）

sub=JOINING:
  计划尚未生成时，收集长机和所有 expectedFollowerIds 的有限、非负基础航程 L_i：
    D_i(n) = L_i + n * 2*pi*R
    I_i(n) = [D_i(n)/V_max, D_i(n)/V_min]
    搜索 earliest_common(I_1(n_1), ..., I_N(n_N))，同时得到各机 n_i
    首次收齐后锁存 T_ref = plan_start_s + earliest_common(...)、loop_counts 和 t_ref_valid=True
  计划已生成时直接复用锁存值；不重算、不后移、不因任何飞机 EXITED 失效
  检查 all_participants_exited(leader_join_exited)
    是 → next_step = CATCHUP
    否 → next_step = JOINING
  输出 cmd.stage=RALLY, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=looseScale, slotScale.scaleRate=0,
       t_ref/t_ref_valid/loop_counts=上述固定计划

sub=CATCHUP:
  检查 all_catchup_ok()
    是 → _catchup_stable_timer += dt_s；若达到 catchup_stable_s → next_step=LOOSE，计时器清零
    否 → _catchup_stable_timer = 0；next_step=CATCHUP
  输出 cmd.stage=RALLY, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=looseScale, slotScale.scaleRate=0

sub=LOOSE:
  检查 all_followers_ok(convergenceRadius_m)（posErr_m 此时为到松散槽位的误差）
    是 → _stable_timer += dt_s；若达到 stableHold_s → next_step=COMPRESS，_stable_timer=0
    否 → _stable_timer = 0；next_step=LOOSE
  输出 cmd.stage=RALLY, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=looseScale, slotScale.scaleRate=0

sub=COMPRESS:
  _compress_elapsed += dt_s
  scale = looseScale - (looseScale-1.0) × (_compress_elapsed / compressTime_s)
  若 scale <= 1.0:
    scale = 1.0
    scaleRate = 0.0          # 已到终值，清零速率；避免负值前馈持续驱动 SlotGeometry
  否则:
    scaleRate = -(looseScale-1.0) / compressTime_s
  若 scale==1.0 且 all_followers_ok(tightRadius_m):
    next_stage=HOLD, next_step=JOINING(0), y.rallyCompleted=True
  否则:
    next_stage=RALLY, next_step=COMPRESS
  输出 cmd.stage=next_stage, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=scale, slotScale.scaleRate=scaleRate
```

**`reset`**：清零阶段计时器和 `_rally_started`，并原子清除
`_plan_start_s/_t_ref/_loop_counts/_plan_ready`，不改配置。
实体 `reset()` 同拍还会清除僚机入站输出锁存和 `RallyJoinPos` 的公切线、基础航程、圈数及跨点状态；
`cmd`/`slotScale` 由下一次 `step()` 按新生命周期输入重新写出。

测试用例：

- expectedFollowerIds 为空 → 各门控立即通过、计时器立即累加（测试用）
- 期望列表非空但 followerStates 为空 → 门控 False，不切换
- 某机超时（断链）→ is_valid=False，计时器冻结
- JOINING：某机仍在 FLYING/LOITERING（未 EXITED）→ all_participants_exited=False，不进 CATCHUP；
  已 EXITED 的机不因随后断链被撤销
- CATCHUP：位置或航向任一超阈值 → `_catchup_stable_timer` 清零；两者同时达标并连续满足
  `catchup_stable_s` → 进 LOOSE
- LOOSE 阶段某机 posErr 不满足 → `_stable_timer` 重置
- COMPRESS 过程 scale 线性变化验证；`scale==1.0` 且槽位误差 < `tightRadius_m` → 完成并置 HOLD

---

### 5.2 Outbound/FollowerBroadcast — 僚机广播位置

**文件**：`units/process/outbound/follower_broadcast.py`

作用：僚机将本机当前位置、到目标槽位的误差、是否到达 M_i 打包广播，供长机做收敛判定。

```python
@dataclass
class FollowerBroadcastInitS(OutboundInitS):
    # 继承 selfId: str, netWork: list[NetWorkS]
    leaderId: str = ""  # 长机节点 ID，明确指定发送目标，不依赖 netWork 推断角色

@dataclass
class FollowerBroadcastInputS(OutboundInputS):
    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    selfCmd: MotionProfS = None       # 端口 → Context.selfCmd，当前目标（用于计算 posErr_m）
    selfArrived: int = 0              # 兼容旧协议；新协议以 rally_state 为准，由 _update_outbound() 从 RallyJoinPos.state==EXITED 派生
    rally_state: str = "FLYING"       # 集结汇合状态：FLYING / LOITERING / EXITED（来自 RallyJoinPos.state）
    planned_path_length_m: float = -1.0  # 本机锁存的基础水平航程 L_i，来自 RallyJoinPos
    reached_slot_once: bool = False   # 是否已至少一次路过 M_i，来自 RallyJoinPos.reached_slot_once

@dataclass
class FollowerBroadcastOutputS(OutboundOutputS):
    pass  # 复用 outbox
```

**Topic**：`formation.follower_status`（区别于长机广播 `formation.leader`，供 FollowerStatus 按 topic 过滤）

**Payload 字段**（序列化顺序固定）：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `id` | str | 本机节点 ID |
| `pos_east` | float | 实际位置东向，米 |
| `pos_north` | float | 实际位置北向，米 |
| `pos_h` | float | 实际高度，米 |
| `pos_err_m` | float | 到当前目标的合距离，米 |
| `heading_err_rad` | float | 当前航向与目标航向之差的绝对值，弧度 |
| `arrived` | int | 1=已到达 M_i（锁存），0=未到达 |
| `rally_state` | str | 集结汇合状态：FLYING / LOITERING / EXITED |
| `planned_path_length_m` | float | 开始 JOINING 后锁存的基础水平航程 L_i，米 |
| `reached_slot_once` | bool | 是否已至少一次路过 M_i，汇合过程诊断量 |

实现：`posErr_m = \|selfState.pos - selfCmd.pos\|`；`arrived = u.selfArrived`；按上表打包为 envelope，topic=`formation.follower_status`，target 为 `cfg.leaderId`（init 时由配置显式传入，不依赖 netWork 推断角色）。`planned_path_length_m` 只有有限且非负时才参与长机计划。

**`init()` 校验**：`cfg.leaderId == ""` 时抛 `ValueError("FollowerBroadcast: leaderId must not be empty")`，防止消息无目标节默认广播。

测试：selfArrived=0 时广播 arrived=0；selfArrived 置 1 后即使 posErr 变大仍广播 arrived=1；posErr 计算正确。

---

### 5.3 Inbound/FollowerStatus — 长机解析僚机回报

**文件**：`units/process/inbound/follower_status.py`

作用：长机从 inbox 解析各僚机广播，写入 `Context.followerStates`。

```python
@dataclass
class FollowerStatusInitS(InboundInitS):
    pass

@dataclass
class FollowerStatusInputS(InboundInputS):
    # 继承 inbox: list[MessageEnvelope]
    now_s: float = 0.0   # 当前仿真时间，写入 FollowerStateS.lastUpdate_s；由实体从边界注入

@dataclass
class FollowerStatusOutputS(InboundOutputS):
    followerStates: list[FollowerStateS] = None  # 端口 → Context.followerStates
```

实现：遍历 inbox，按 `topic == "formation.follower_status"` 过滤（排除长机自身的 `formation.leader` 等其他报文）；**以 `envelope.source` 作为节点 ID 做列表查找和写入**，不使用 payload 中的 `id` 字段（payload id 不可信，伪造后会污染状态表）；原地更新对应 `FollowerStateS` 的其余字段（pos/posErr_m/arrived/valid=True/lastUpdate_s=now_s，`entry.id = envelope.source`）；不在列表中的 source 追加新条目；断链帧不操作已有条目（保留 posErr_m 但不更新 lastUpdate_s，valid 保留上一帧，长机侧由超时检测处理）。

测试：

- inbox 含 2 僚机广播 → followerStates 各字段（含 valid/lastUpdate_s）解析正确
- inbox 为空（断链）→ followerStates 的 lastUpdate_s 不更新，valid 不变（由超时逻辑在 Rally 侧处理）
- inbox 含非僚机报文 → 正确过滤

---

### 5.4 Outbound/RallyLeaderBroadcast — 集结长机广播

**文件**：`units/process/outbound/rally_leader_broadcast.py`

作用：唯一的长机广播实现，普通保持与集结共用。它把长机实际状态 `selfState`、长机有效指令 `leaderCmd`、`cmd.stage/pattern/step`、`slotScale.scale/scaleRate`、固定计划 `t_ref/t_ref_valid/loop_counts` 打入同一条 `formation.leader` envelope；`leaderCmd` 未绑定时回退 `selfState`。普通保持场景也使用该类，只是 `slotScale` 恒为 `scale=1.0/scaleRate=0.0`，`t_ref_valid=False`，对保持编队无影响。

payload 结构：

```python
payload = {
    "leader_state": _motion_payload(u.selfState),
    "cmd": {
        "stage": int(u.cmd.stage),
        "pattern": int(u.cmd.pattern),
        "step": int(u.cmd.step),
        "leader": _motion_payload(u.leaderCmd or u.selfState),
    },
    "slot_scale": {"scale": u.slotScale.scale, "scale_rate": u.slotScale.scaleRate},
    "t_ref": u.t_ref,
    "t_ref_valid": u.t_ref_valid,
    "loop_counts": u.loop_counts,
}
```

InitS/OutputS 直接复用通用基类，无需新建：`RallyLeaderBroadcastInitS = OutboundInitS`；`RallyLeaderBroadcastOutputS = OutboundOutputS`（含 `outbox`）。

```python
@dataclass
class RallyLeaderBroadcastInputS(OutboundInputS):
    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    leaderCmd: MotionProfS = None      # 可选端口 → 长机有效运动指令；缺省回退 selfState
    slotScale: RallySlotScaleS = None   # 端口 → Context.slotScale（含 scale + scaleRate）
    t_ref: float = 0.0
    t_ref_valid: bool = False
    loop_counts: dict[str, int] = field(default_factory=dict)
```

---

### 5.5 Inbound/RallyLeaderFollower — 集结僚机解析长机广播

**文件**：`units/process/inbound/rally_leader_follower.py`

作用：唯一的长机广播入站解析实现，普通保持与集结共用。它在同一条 `formation.leader` 消息中解析 `leader_state/cmd/slot_scale/t_ref/t_ref_valid/loop_counts`，写入 `Context.leaderState/cmd/slotScale/rally_t_ref/rally_t_ref_valid/rally_loop_counts`，并把 `cmd["leader"]` 写入对象组边界持有的 `leaderCmd` 端口；旧格式缺少 `cmd["leader"]` 时，`leaderCmd` 回退为 `leader_state`。缺少 `slot_scale` 的旧格式消息按 `scale=1.0/scaleRate=0.0` 兜底；计划字段不完整或非法时整条消息不提交，避免冷启动误用半截计划。

```python
@dataclass
class RallyLeaderFollowerOutputS(InboundOutputS):
    # 继承 leaderState: MotionProfS, cmd: FormSnapshotS
    leaderCmd: MotionProfS = None      # 可选端口 → 对象组边界持有，供 SlotGeometry 建系
    slotScale: RallySlotScaleS = None   # 端口 → Context.slotScale
    t_ref: float = 0.0
    t_ref_valid: bool = False
    loopCounts: dict[str, int] = field(default_factory=dict)
```

**多消息胜出规则**：同帧 inbox 中可能有多条 `formation.leader` 消息（重发或乱序）。遍历 inbox 时**按序处理，每条完整有效消息均覆盖写入** `leaderState/leaderCmd/cmd/slotScale`，故最后一条有效消息最终胜出。`leaderState/leaderCmd/cmd/slotScale` 四个字段必须来自**同一条消息**，不允许跨消息拼装，避免字段不一致。

解析逻辑（先完整临时解析，异常报文静默丢弃）：

```python
for envelope in inbox:
    if envelope.topic != "formation.leader" or not isinstance(envelope.payload, dict):
        continue
    try:
        parsed = _parse_leader_broadcast(envelope.payload)
        # parsed 内已严格校验运动数值有限性、cmd 整数类型和 FormStageE 枚举、
        # slot_scale/T_ref 有限性、t_ref_valid 布尔类型及 loop_counts 键值类型。
    except (TypeError, ValueError, OverflowError):
        continue

    # 只有全部字段均有效后才复制到绑定输出；任何异常都不会留下新旧混合状态。
    copy_motion(parsed.leader_state, y.leaderState)
    copy_snapshot(parsed.cmd, y.cmd)
    if y.leaderCmd is not None:
        copy_motion(parsed.leader_cmd, y.leaderCmd)
    copy_rally_slot_scale(parsed.slot_scale, y.slotScale)
    y.t_ref = parsed.t_ref
    y.t_ref_valid = parsed.t_ref_valid
    y.loopCounts.clear(); y.loopCounts.update(parsed.loop_counts)
```

缺少 `slot_scale` 键时仍按旧格式默认 `scale=1.0/scaleRate=0.0`；但字段存在却类型非法、包含非有限数，
或运动/命令/计划任一字段非法时，整条消息不提交。`reset()` 把最近绑定的
`leaderState/leaderCmd/cmd/slotScale/t_ref/t_ref_valid/loopCounts` 全部恢复默认值，保证实体复位后的空 inbox
不能把旧计划重新写回 Context。

---

## 六、新增算法组单元

### 6.1 PosCalc/RallyJoinPos — 切入盘旋圆汇合（原 RallyApproach，已整体替换）

> 本节原描述 `RallyApproach`（APPROACH 子阶段直飞预分配目标点 M_i，水平/垂向各自独立比例控制、无盘旋
> 协调）。`RallyJoinPos`（切线进圆 + 盘旋协调汇合）设计定稿后，`rally_approach.py` 已整个删除，JOINING
> 阶段（`cmd.step==RallyPhaseE.JOINING`）完全由 `RallyJoinPos` 负责位置解算，不存在替代关系之外的共存。

**文件**：`units/algo/pos_calc/rally_join_pos.py`

作用：长机与僚机共用的集结位置解算单元，内部 `STANDBY → FLYING → LOITERING → EXITED` 状态机：本地待命盘旋 → 待命圆切出并沿定向公切线进入集结圆 → 以固定计划协调速度完成全航程 → 按分配圈数从 `M_i` 沿任务航向切出。`FLYING` 的 `ARC_TO_TANGENT/LINE_TO_RALLY_ENTRY` 是私有子阶段，对外均保持 `FLYING`。完整算法（两圆公切线、切入触发半径、基础航程、固定 `T_ref` 和按圈切出）见「二、总体策略与状态机」第二步；本节只列初始化/输入端口的 API 形状：

```python
@dataclass
class RallyJoinPosInitS(PosCalcInitS):
    loose_slot: PosInEarthS = field(default_factory=PosInEarthS)  # 本机固定松散目标点 M_i，同时是盘旋圆上的切出点
    approach_speed_mps: float = 20.0   # 飞向切入点 T 的速度
    slow_radius_m: float = 0.0         # 近场降速半径；>0 时在此范围内线性减速
    arrival_radius_m: float = 100.0    # 到达切入点、转入圆弧飞行的触发距离
    loiter_radius_m: float = 200.0     # 盘旋圆半径（固定）
    loiter_speed_min_mps: float = 14.0
    loiter_speed_max_mps: float = 25.0
    mission_heading_rad: float = 0.0   # 切出后的飞行方向（弧度，东向为 0）
    mission_speed_mps: float = 20.0    # 切出后的飞行速度
    v_up_min_mps: float = -3.0         # 天向速度下限（来自 velCmdLimit.verticalMin）
    v_up_max_mps: float = 3.0          # 天向速度上限（来自 velCmdLimit.verticalMax）
    control_period_s: float = 0.05     # 控制周期；用于校验切入圆弧触发半径的离散步进安全余量
    standby_altitude_m: float | None = None  # 待命和转移阶段的本机分层高度

@dataclass
class RallyJoinPosInputS(PosCalcInputS):
    # 继承 selfState: MotionProfS → Context.selfState
    t_ref: float = 0.0        # 已锁存的全队公共到达绝对时刻
    t_ref_valid: bool = False # 首次有效计划到达前为 False
    assigned_loops: int = 0   # 本机在首次到达 M_i 后需追加的完整圈数
    t_now: float = 0.0        # 当前仿真时间
    standby: bool = False     # True 时保持本地待命圆；False 时离开待命并一次性规划转移路径

# 输出复用 PosCalcOutputS；state/planned_path_length_m/remaining_path_length_m/remaining_loops/reached_slot_once 为只读属性
```

计划有效后每拍按实际子阶段计算剩余航程：

```text
D_remaining = 当前子阶段剩余长度 + 后续固定路径段长度 + remaining_loops * 2*pi*R
V_cmd = clamp(D_remaining / (T_ref - t_now), V_min, V_max)
```

当 `T_ref - t_now <= 0` 时速度退化为 `V_max`，仅作离散计算兜底，不重新规划或追加圈数。

`init()` 会调用 `validate_capture_geometry()` 校验 `loiter_radius_m`/`arrival_radius_m` 相对
`approach_speed_mps`/`loiter_speed_min_mps`/`control_period_s` 是否留有足够的离散步进安全余量，不合法
直接抛 `ValueError`（`_ConfigLoader.validate()` 在配置加载阶段复用同一函数提前校验，见第二步 JOINING
阶段说明）。

---

### 6.2 PosCalc/SlotGeometry — 带缩放的槽位几何

**文件**：`units/algo/pos_calc/slot_geometry.py`

作用：`SlotGeometry` 是唯一的僚机实时槽位几何实现。槽位使用三维苏联式航迹系 FUR（`x` 前、`y` 上法向、`z` 右）；普通保持场景不绑定 `slotScale` 或使用默认 `scale=1.0/scaleRate=0.0`，输出标准槽位；集结 CATCHUP/LOOSE/COMPRESS/HOLD 绑定 `Context.slotScale`，在同一 `step()` 中对队形平面内的 `x/z` 做缩放，并按 `scaleRate` 追加压缩速度前馈，`y` 不缩放。COMPRESS 阶段 scale 随时间线性减小，目标位置随之平滑收敛。

```python
@dataclass
class SlotGeometryInitS(PosCalcInitS):
    selfId: str = ""
    formPat: list[str] = field(default_factory=list)
    formPos: list[list[FormPosS]] = field(default_factory=list)

@dataclass
class SlotGeometryInputS(PosCalcInputS):
    leaderState: MotionProfS = None
    leaderCmd: MotionProfS = None  # 可选有效指令帧；FUR 方向与速度优先取它
    cmd: FormSnapshotS = None
    slotScale: RallySlotScaleS = None  # 可选端口；None 时按 scale=1.0/scaleRate=0.0 处理

# 输出复用 PosCalcOutputS（selfCmd → Context.selfCmd），无需新增 OutputS 类
```

**实现方式**：槽位查找、三维 FUR 变换、转弯速度前馈和缩放处理都在 `SlotGeometry.step()` 内完成，不再维护单独的缩放槽位文件。位置原点固定取 `leaderState.pos`；FUR 基优先由 `leaderCmd` 的三维地速生成，无有效水平航迹时回退 `leaderState`，两者都退化时明确采用东向平飞 FUR 兜底。

```python
class SlotGeometry(PosCalcBase):
    def step(self, u: SlotGeometryInputS, y: PosCalcOutputS) -> None:
        frame, basis = select_frame_and_fur_basis(u.leaderCmd, u.leaderState)
        sx, sy, sz, vfx, vfy, vfz = smooth_slot_in_fur(...)
        scale = u.slotScale.scale if u.slotScale else 1.0
        scaleRate = u.slotScale.scaleRate if u.slotScale else 0.0

        # x/z 是队形平面尺寸，y 是不缩放的上法向间隔；三轴一起由 FUR 转到 ENU。
        a, y_up, b = scale * sx, sy, scale * sz
        y.selfCmd.pos = u.leaderState.pos + C_FUR_to_ENU @ [a, y_up, b]

        # theta 在本拍视为常值；omega=dVPsi 左转为正，FUR 的 z 轴向右。
        v_yaw_fur = [b*omega*cos(theta), -b*omega*sin(theta),
                     (-a*cos(theta) + y_up*sin(theta))*omega]
        v_transition_fur = [scale*vfx + scaleRate*sx, vfy,
                            scale*vfz + scaleRate*sz]
        y.selfCmd.v = frame.v + C_FUR_to_ENU @ (v_yaw_fur + v_transition_fur)
        y.selfCmd.v.dVPsi = omega
```

`slotScale=None` 或 `scale=1.0` 且 `scaleRate=0` 时输出标准槽位；LOOSE 阶段 `scale=looseScale>1, scaleRate=0`，FUR 的 `x/z` 放大但无压缩速度项；COMPRESS 期间 `scaleRate<0` 自动添加向内的速度前馈。`y` 始终不参与缩放，但在爬升/下降时它是上法向而非世界天向，因此 ENU 高度差不再简单等于 `slot.y`；平飞时才退化为该关系。

测试：

- slotScale 未绑定或 scale=1.0 → pos/v 结果与标准槽位相同
- scale=2.0 → FUR 的 `x/z` 偏置扩大一倍，`y` 不变，速度前馈按同一轴义同步缩放
- scale 从 2.0 线性减到 1.0 → 目标在三维 ENU 中平滑收敛，压缩速度由 FUR 的 `x/z` 映射得到
- 非零爬升角 → `x/y` 随倾角旋转、`z` 保持水平右侧向；左右转偏航刚体速度严格镜像

---

### 6.3 CATCHUP 阶段的位置解算：无专属单元

CATCHUP（step=1）不再有专属的 PosCalc 单元。曾经存在的 `CatchupAlign`（`units/algo/pos_calc/catchup_align.py`）已删除——它当年把飞机投影到过 M_i 点、平行任务航向的"杆"上，人为将前向位置误差钉为 0，再单独用速度调制追赶，理由是当时的 `PidCompose` 按本机自身航迹系投影位置误差，目标落在机尾方向时可能形成横侧向正反馈。`PidCompose` 后来改为按目标（`selfCmd`）自身航迹系投影位置误差（见 6.4 之前 `PidCompose` 一节及 [pid_compose.py:110-121](../../src/algorithm/units/algo/pos_track/pid_compose.py#L110-L121)），横侧向切入由下层控制律统一处理；前向通道仍按位置误差修正速度，正值 `forwardMin` 负责禁止负速度指令。`CatchupAlign` 的投影/锁航向/速度调制因此变成与下层控制律重复的多余逻辑，故删除。

CATCHUP 现在直接复用 **6.2 节的 `SlotGeometry`**：给出真实槽位位置 + 槽位自身速度前馈，交给 `PidCompose` 的前向 PPI 外环闭环（落后加速、超前减速、到达后跟随槽位速度，`forwardMin/forwardMax` 限幅避免倒飞）。`posErr_m` 因此和 LOOSE 阶段一样，由 `FollowerBroadcast` 统一算 `dist3d(selfState, selfCmd)`，不再需要 `pos_err_m_override` 这类特判接口。

---

## 七、新增实体

### 7.1 RallyLeaderEntity

**文件**：`entity/leader_follower_rally/leader.py`

#### 7.1.1 使用的单元子类

> 本节曾描述两条独立航线贯穿 RALLY 全程的旧架构，
> 那是 `RallyJoinPos`（切线进圆汇合）出现之前的设计，已和当前实现不符——JOINING 阶段（step=0）现在完全
> 由 `RallyJoinPos` 负责位置解算，不经过 TraPlan/RouteInterp；`LeaderRoute`+`RouteInterp` 只在 CATCHUP
> 及之后（step>=1）用于沿统一 `cfg.route` 飞行。以下按实际实现更正。

| 单元 | 子类 |
| --- | --- |
| 收消息 Inbound | FollowerStatus（解析僚机回报） |
| 任务编排 FormationTask | Rally（集结状态机） |
| 汇合位置解算（STANDBY / JOINING，step=0） | RallyJoinPos（待命盘旋→公切线转移→集结圆盘旋→切出，见第二步） |
| 轨迹规划 TraPlan（仅 CATCHUP 及之后，step>=1 / HOLD） | LeaderRoute（统一 `cfg.route`） |
| 位置解算 PosCalc（仅 CATCHUP 及之后） | RouteInterp（复用现有） |
| 跟踪 PosTrack | PidCompose（复用现有） |
| 发消息 Outbound | RallyLeaderBroadcast（统一长机广播，含 slotScale/t_ref/t_ref_valid/loop_counts） |

#### 7.1.2 调用顺序（一拍 step）

```text
收消息(FollowerStatus)                    ← 解析僚机回报 → Context.followerStates
→ 任务编排(Rally)                         ← 读 followerStates/remote → 写 cmd + slotScale + 固定协调计划
→ 按 cmd.step 分流：
    STANDBY / JOINING（step=0）：汇合位置解算(RallyJoinPos) ← 待命盘旋/公切线转移/集结圆盘旋/切出，不经 TraPlan/RouteInterp
    CATCHUP 及之后（step>=1）/HOLD：轨迹规划(LeaderRoute) → 位置解算(RouteInterp)  ← 沿统一 route 飞行
→ 跟踪(PidCompose)                        ← 复用，两条分支共用
→ 发消息(RallyLeaderBroadcast)            ← 广播 selfState + cmd + slotScale + t_ref/t_ref_valid/loop_counts
```

`step()` 中的分流逻辑（L1 职责，摘自 `leader.py::step`）：

```python
stage = self.cxt.cmd.stage
step = self.cxt.cmd.step

if stage == FormStageE.NONE:  # 跳过位置解算 / PidCompose，直接输出当前位置零速
    copy_position(self.cxt.selfState.pos, self.cxt.selfCmd.pos)
    zero_velocity(self.cxt.selfCmd.v)
    zero_acceleration(self.cxt.selfAccCmd)
    self._outbound.step(self._outbound_u, self._outbound_y)
    fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
    return

if stage == FormStageE.RALLY and step == RallyPhaseE.JOINING:
    # JOINING 阶段：长机平等参与，也飞向自己的松散点（队形中心 A）
    self._rally_join.step(self._rally_join_u, self._pos_calc_y)
    self._pos_track.step(self._pos_track_u, self._pos_track_y)
else:
    # RALLY step>=1（CATCHUP/LOOSE/COMPRESS）或 HOLD：长机沿统一航线飞行
    self._tra_plan_mission.step(self._tra_plan_u, self._tra_plan_y)
    self._pos_calc.step(self._pos_calc_u, self._pos_calc_y)
    self._pos_track.step(self._pos_track_u, self._pos_track_y)
```

#### 7.1.3 初始化（init）关键点

- 实例化一个 `LeaderRoute`：`_tra_plan_mission(cfg.route)`，只服务 CATCHUP 及之后；JOINING 阶段的
  `RallyJoinPos` 不需要 TraPlan/RouteInterp，直接用 `RallyJoinPosInitS(loose_slot=A, ...)` 初始化
- `FollowerStatus` 单元的 **`inbox` 端口绑定到 `EntityInputS.inbox`**（每帧由边界层注入，不可遗漏，否则长机永远收不到僚机消息）
- `slotScale` 端口绑定到 `Context.slotScale`
- `followerStates` 端口绑定到 `Context.followerStates`
- `RallyTaskInitS.dt_s` 与 `cfg.control_period_s` 保持一致，init 时传入
- 每拍 `step()` 中需将边界输入的仿真时间注入两个单元：`follower_status_u.now_s = now` 和 `rally_u.now_s = now`
- `rallyCompleted` 不进 Context，实体直接读 `_task_y.rallyCompleted`（OutputS 每拍重写，无需 Context 中继）；`expectedFollowerIds` 在实体侧持有一份副本（从 `cfg.rally_cfg.expectedFollowerIds` 复制），供 `FormationAnalysisS` 计算时使用，不从 Rally 单元内部读取

**当前有效航线关键点**：

- `A = route[0].pos`：集结中心，也是掌机的 JOINING 目标位置。
- `A1 = route[1].pos`：首段终点，用于推导集结队形朝向和切出后的任务航向。
- `route[2...]`：后续任务航点，由 `LeaderRoute` 在 CATCHUP 及以后继续推进。

掌机的 `RallyJoinPosInitS.loose_slot` 设为 A；`mission_heading_rad` 取第一航段方向：

```python
A = cfg.route[0].pos
A1 = cfg.route[1].pos
mission_heading_rad = math.atan2(A1.north - A.north, A1.east - A.east)
```

`_ConfigLoader.validate()` 和两个集结实体都会校验航线至少包含两个航点，且首段水平长度不能退化为零。
由于集结与任务飞行读取同一条当前有效 `route`，不再需要两条航线之间的位置和方向一致性校验。采用避障覆盖航线后，集结中心、方向、高度分层和 GUI 几何会随有效航线同步更新，这是为了避免集结阶段继续沿已判定存在障碍的原始航线飞行。

---

### 7.2 RallyFollowerEntity

**文件**：`entity/leader_follower_rally/follower.py`

#### 7.2.1 使用的单元子类

| 单元 | 子类 |
| --- | --- |
| 收消息 Inbound | RallyLeaderFollower（统一长机广播解析，含 slotScale/t_ref/t_ref_valid/loop_counts） |
| 任务编排 FormationTask | 不使用（模态来自长机广播） |
| 轨迹规划 TraPlan | Noop（复用） |
| 位置解算 PosCalc | RallyJoinPos（STANDBY/JOINING）/ SlotGeometry（CATCHUP/LOOSE/COMPRESS，同一算法） |
| 跟踪 PosTrack | PidCompose（复用） |
| 发消息 Outbound | FollowerBroadcast（回报位置与状态） |

#### 7.2.2 调用顺序（一拍 step）

```text
收消息(RallyLeaderFollower)               ← 解析长机广播 → leaderState/leaderCmd + cmd + slotScale + t_ref/t_ref_valid/loop_counts
→ [轨迹规划(Noop) 空策略]
→ 位置解算（按 cmd.stage + cmd.step 路由）
    cmd.stage==NONE:                        跳过 PosCalc，直接输出零速保持当前位置（不触发跟踪）
    cmd.stage==STANDBY:                     RallyJoinPos       ← 本地待命圆盘旋
    cmd.stage==RALLY, cmd.step==0:          RallyJoinPos       ← JOINING：公切线转移 / 集结圆盘旋 / 切出
    cmd.stage==RALLY, cmd.step>=1 / HOLD:   SlotGeometry ← CATCHUP/LOOSE/COMPRESS/HOLD：三维槽位跟随
→ 跟踪(PidCompose)（NONE 时跳过）
→ 发消息(FollowerBroadcast)               ← 回报位置 + posErr + arrived
```

PosCalc 切换逻辑（L1 职责，摘自 `follower.py::step`；替换本节曾描述的"RallyApproach 直飞 M_i + `_self_arrived`
锁存到达"旧流程——那是 `RallyJoinPos` 出现之前的设计，已和当前实现不符）：

```python
if u.selfState is not None:
    copy_motion(u.selfState, self.cxt.selfState)
previous_stage = self.cxt.cmd.stage
self._inbox.clear(); self._inbox.extend(u.inbox)

self._inbound.step(self._inbound_u, self._inbound_y)          # 原子解析状态、命令、缩放和固定计划
self.cxt.rally_t_ref = self._inbound_y.t_ref                   # T_ref 仅供 RallyJoinPos 全航程调速
self.cxt.rally_t_ref_valid = self._inbound_y.t_ref_valid
self.cxt.rally_loop_counts = dict(self._inbound_y.loopCounts)
self._tra_plan.step(self._tra_plan_u, self._tra_plan_y)        # Noop 空策略

stage = self.cxt.cmd.stage

if stage == FormStageE.NONE:
    # 低层兼容停控分支会清理 RallyJoinPos 状态，但不构成受支持的中断或二次集结协议；
    # 新生命周期必须由控制器先对全部实体执行显式 reset()
    if previous_stage in (FormStageE.RALLY, FormStageE.HOLD):
        self._rally_join.reset()
    copy_position(self.cxt.selfState.pos, self.cxt.selfCmd.pos)  # 逐字段复制，避免别名
    zero_velocity(self.cxt.selfCmd.v)
    zero_acceleration(self.cxt.selfAccCmd)
    self._update_outbound()   # rally_state/planned_path_length_m/reached_slot_once/selfArrived 一并写入出站端口
    self._outbound.step(self._outbound_u, self._outbound_y)
    fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
    return

if stage == FormStageE.STANDBY or (
    stage == FormStageE.RALLY and self.cxt.cmd.step == RallyPhaseE.JOINING
):
    # STANDBY/JOINING：RallyJoinPos 内部待命盘旋→公切线转移→集结圆盘旋→切出
    self._rally_join_u.standby = stage == FormStageE.STANDBY
    self._rally_join_u.t_ref = self.cxt.rally_t_ref
    self._rally_join_u.t_ref_valid = self.cxt.rally_t_ref_valid
    self._rally_join_u.t_now = u.now_s
    self._rally_join_u.assigned_loops = self.cxt.rally_loop_counts.get(self._self_id, 0)
    self._rally_join.step(self._rally_join_u, self._pos_calc_y)
    self._pos_track.step(self._pos_track_u, self._pos_track_y)
else:
    # RALLY step>=1（CATCHUP/LOOSE/COMPRESS）或 HOLD：三维槽位跟随
    # CATCHUP 与 LOOSE/COMPRESS 用同一套算法，二者区别只在 Rally 任务的阶段门控上
    self._pos_calc_slot.step(self._slot_u, self._pos_calc_y)
    self._pos_track.step(self._pos_track_u, self._pos_track_y)
self._update_outbound()
self._outbound.step(self._outbound_u, self._outbound_y)
fill_output(self.cxt, self._pos_track_diag, self._outbox, y)
```

`_update_outbound()`（每拍把 `RallyJoinPos` 状态同步到出站端口，供长机收集基础航程和到达判定）：

```python
self._outbound_u.rally_state = self._rally_join.state
self._outbound_u.planned_path_length_m = self._rally_join.planned_path_length_m
self._outbound_u.reached_slot_once = self._rally_join.reached_slot_once
self._outbound_u.selfArrived = 1 if self._rally_join.state == RALLY_STATE_EXITED else 0
```

> **说明**：`cmd.step` 是长机广播的系统级状态（`RallyPhaseE.JOINING/CATCHUP/LOOSE/COMPRESS`），驱动本机
> PosCalc 单元切换，本机是否"到达"改由 `RallyJoinPos.state == RALLY_STATE_EXITED`（切出）和
> `reached_slot_once`（是否已路过 M_i 一次）两个信号表达，不再有独立的 `_self_arrived` 锁存字段——
> `selfArrived` 直接从 `_rally_join.state` 派生；`reached_slot_once` 保留为汇合过程诊断量。长机仅用
> `planned_path_length_m` 生成一次固定计划，之后由 `T_ref` 调速、由 `loop_counts` 决定切出次数。

#### 7.2.3 初始化（init）关键点

- `loose_slot`（M_i）：在 `init()` 中调用 `rally_loose_target(A, mission_heading_rad, rally_cfg.looseScale, slot)`
  计算（`A=route[0].pos`，`mission_heading_rad` 由 `route_heading_rad(route)` 推导第一航段
  方向，`slot` 由 `resolve_formation_slot(cfg.commInit, rally_cfg.targetPattern, cfg.selfInit.id)` 按目标队形
  索引查表得到），高度偏置固定为 `slot.y`（不随 `looseScale` 扩展），**不从配置字段读取**（逐节点
  `rally_target` 配置字段已移除）。该函数只生成基于首段水平航向、用于派生集结圆心的 ENU 水平
  M_i 几何，等价于任务航向对齐、倾角为零的合成平飞 FUR；非零真实航迹倾角不会旋转 `slot.x/y`，
  不能将它复用于实时三维 FUR 槽位
- `loiter_speed_min/max_mps`：由 `loiter_speed_bounds(cfg.velCmdLimit)` 推导（未显式配置的一侧退回默认
  14/25 m/s，并校验两者不反序，见第二步 JOINING 阶段说明）
- `RallyJoinPosInitS.control_period_s`：传入 `cfg.control_period_s`，用于校验切入圆弧触发半径的离散步进安全余量
- `RallyJoinPos` 在 `stage` 变为 `NONE` 时为兼容停控清除内部相位状态（`FLYING/
  LOITERING/EXITED`、`reached_slot_once` 等）；任务生命周期仍保持锁存，未经全部实体显式 `reset()` 不接受新 RALLY
- **端口绑定（不可遗漏）**：
  - `RallyLeaderFollower` Inbound 的 `leaderCmd` 输出 → 对象组边界持有的 `_leader_cmd`，供 `SlotGeometryInputS.leaderCmd` 使用；`slotScale`/`t_ref`/`t_ref_valid`/`loopCounts` 输出 → 每拍写入 `Context.slotScale`/`rally_t_ref`/`rally_t_ref_valid`/`rally_loop_counts`
  - `RallyJoinPos` 的 `selfState` 端口在 init 时绑定到 `Context.selfState`；`t_ref`/`t_ref_valid`/`t_now` 不是一次性端口绑定，
    而是每拍在 `step()` 里从 `Context.rally_t_ref`/`rally_t_ref_valid`/`rally_loop_counts`/`u.now_s` 赋值；已锁存的计划不会被后续广播改变
  - `SlotGeometry` 的 `leaderState`/`leaderCmd`/`cmd`/`slotScale` 端口 → `Context.leaderState`/对象组 `_leader_cmd`/`Context.cmd`/`Context.slotScale`（CATCHUP 与 LOOSE/COMPRESS 共用）
  - `FollowerBroadcast` 的 `rally_state`/`planned_path_length_m`/`reached_slot_once`/`selfArrived` 端口由 `_update_outbound()` 每拍写入

---

## 八、配置与边界类型扩展

### 8.1 EntityInputS 扩展

```python
@dataclass
class EntityInputS:
    # ... 已有字段 ...
    now_s: float = 0.0   # 当前仿真时间戳（秒）；由仿真框架每帧注入，用于僚机报文超时检测
```

实体在 `step()` 中将 `now_s` 注入到需要时钟的单元（`FollowerStatusInputS.now_s`、`RallyTaskInputS.now_s`）。

### 8.2 EntityInitS 扩展

```python
@dataclass
class EntityInitS:
    # ... 已有字段 ...
    route: list[WayPointInputS] = field(default_factory=list)  # 统一航线；[0]=集结中心，首段确定集结方向
    rally_cfg: RallyTaskInitS = field(default_factory=RallyTaskInitS)  # 集结参数
    rally_approach_speed_mps: float = 20.0  # 僚机飞向 M_i 的速度
    rally_leader_id: str = ""               # 僚机回报消息的发送目标（来自节点配置 leader_id）
```

### 8.3 配置文件扩展（JSON）

```json
{
  "route_file": "element/rally_demo_mission_route.json",
  "rally_cfg": {
    "loose_scale": 3.0,
    "convergence_radius_m": 30.0,
    "stable_hold_s": 4.0,
    "compress_time_s": 20.0,
    "tight_radius_m": 5.0,
    "stale_timeout_s": 3.0,
    "loiter_radius_m": 200.0,
    "arrival_radius_m": 100.0,
    "catchup_radius_m": 200.0,
    "approach_speed_mps": 20.0
  },
  "formation": {
    "coordinate_system": "x_forward_y_up_z_right",
    "formation_files": [
      "element/formations/triangle_3_aircraft_a01_a03.json"
    ]
  },
  "nodes": [
    {
      "node_id": "A01",
      "role": "rally_leader"
    },
    {
      "node_id": "A02",
      "role": "rally_follower",
      "leader_id": "A01"
    },
    {
      "node_id": "A03",
      "role": "rally_follower",
      "leader_id": "A01"
    }
  ]
}
```

队形文件 `element/formations/triangle_3_aircraft_a01_a03.json`：

```json
{
  "name": "三机三角",
  "slots": [
    { "node_id": "A01", "x_m":   0.0, "y_m": 0.0, "z_m":  0.0 },
    { "node_id": "A02", "x_m": -54.0, "y_m": 0.0, "z_m": -58.0 },
    { "node_id": "A03", "x_m": -54.0, "y_m": 0.0, "z_m":  58.0 }
  ]
}
```

> `route_file` 加载后展开为顶层配置 `route`，并作为初始有效航线；采用避障规划结果后，覆盖航线成为当前有效 `route`。当前有效 `route[0]` 是集结中心，`route[0]→route[1]` 确定集结与出航方向，后续航点用于任务飞行。
>
> **M_i 自动计算**：从机的松散目标点由实体 init 自动推导，无需在配置文件中逐机写死。实体读取 `route[0]`（A）和 `route[1]` 推导 θ，再结合 `formation.formation_files` 展开后的目标队形槽位和 `loose_scale` 计算 M_i；**已不存在逐节点 `rally_target` 配置字段**。
>
> **顶层键名是 `rally_cfg`，不是 `rally`**（历史文档遗留错误，此处已更正）。`expected_follower_ids` **不是**配置字段——参与集结的僚机 ID 由 `_build_rally_task_init` 从 `nodes` 中 `role=="rally_follower"` 的节点自动收集，配置里写了也会被忽略。`target_pattern` 同理：集结只用单队形，`_build_rally_task_init` 恒将 `targetPattern` 置 0（`formPos` 第 0 行），配置里的 `target_pattern` 键当前不参与任何计算。
>
> `approach_speed_mps` 省略时默认 20 m/s（`EntityInitS.rally_approach_speed_mps` 默认值）。天向速度限幅
> 不再来自 `rally_cfg`（旧版 `k_alt`/`v_up_max_mps` 字段已随 `RallyApproach` 一起移除），而是与其余实体
> 共用顶层 `control.velocity_command_limits.vertical_min_mps`/`vertical_max_mps`（见 `configs/rally_demo_5_aircraft.json`
> 实际配置），未配置时退回 `RallyJoinPosInitS` 的默认值 ±3 m/s。

---

## 九、目录结构总览

```text
src/algorithm/
├── context/
│   ├── context.py              ← 扩展：slotScale、followerStates 字段
│   └── leaf_types.py           ← 扩展：RallySlotScaleS、FollowerStateS、FormationAnalysisS
├── entity/
│   ├── leader_follower_hold/   ← 不动
│   └── leader_follower_rally/  ← 新建
│       ├── leader.py           # RallyLeaderEntity
│       └── follower.py         # RallyFollowerEntity
└── units/
    ├── algo/
    │   └── pos_calc/
    │       ├── route_interp.py           ← 不动
    │       ├── slot_geometry.py          ← 扩展：普通槽位 + slotScale 缩放槽位能力（CATCHUP/LOOSE/COMPRESS 共用）
    │       └── rally_join_pos.py         ← 新建（切入盘旋圆汇合，原 rally_approach.py 已整体替换并删除）
    └── process/
        ├── formation_task/
        │   ├── hold.py                   ← 不动
        │   └── rally.py                  ← 新建
        ├── tra_plan/
        │   └── leader_route.py           ← 不动（两个实例在实体里管理）
        ├── outbound/
        │   ├── rally_leader_broadcast.py ← 统一长机广播（保持/集结共用）
        │   └── follower_broadcast.py     ← 新建
        └── inbound/
            ├── rally_leader_follower.py  ← 统一长机广播解析（保持/集结共用）
            └── follower_status.py        ← 新建
```

---

## 十、编队分析输出

仅当 COMPRESS 子阶段自然完成（内部 `_rally_completed` 标志置位）后的第一拍，`RallyLeaderEntity` 计算并输出 `FormationAnalysisS`。`remote==HOLD` 外部强制中断和 `remote==NONE` 复位均不触发分析输出。

`_rally_completed` 由 Rally 单元在 COMPRESS→HOLD 转换时写入输出（`y.rallyCompleted = True`），实体读取后置位本地标志；`remote==NONE` 时实体可清除该显示锁存，但任务仍拒绝重启，只有显式 `reset()` 才开始新的集结生命周期。

**触发条件**：`_rally_completed==True` 且本拍为首次（只输出一拍，后续帧 `formationAnalysis=None`）。实体侧一次性；仿真层把这一拍的值持久锁存到 `SimulationController._formation_completed_analysis`，并通过 `SimulationSnapshot.rally_analysis` 对外提供（详见 11.4 节）。当前 GUI 适配层不透传或展示该分析结果。

集结完成后，`RallyLeaderEntity` 计算并输出 `FormationAnalysisS`：

- 只统计 `expectedFollowerIds` 中存在、且 `valid==True`、且 `(now_s - lastUpdate_s) <= staleTimeout_s` 的条目（排除断链旧状态和非预期节点）
- `validStates = [s for s in followerStates if s.id in expectedFollowerIds and is_valid(s)]`
- 若 `validStates` 为空：`posErrMax_m = NaN`、`posErrRms_m = NaN`、`inPositionCount = 0`（空列表无法取 max/sqrt-mean，统一置 NaN/0 避免 ValueError）
- 否则：`posErrMax_m = max(s.posErr_m for s in validStates)`；`posErrRms_m = sqrt(mean(s.posErr_m² for s in validStates))`；`inPositionCount = count(s.posErr_m < tightRadius_m for s in validStates)`
- `totalCount = len(expectedFollowerIds)`（期望总数，不受在线状态影响）

`FormationAnalysisS` 通过 `EntityOutputS` 携带出，不进 `Context`（仅诊断/日志用）。

`EntityOutputS` 扩展：

```python
@dataclass
class EntityOutputS:
    # ... 已有字段 ...
    formationAnalysis: FormationAnalysisS = None  # 仅集结完成首帧非 None；仿真层须另行锁存
```

---

## 十一、打桩与集成接入

本节汇总 `sim_control.py`、`main_window.py` 以及 `configs/` 目录所需的改动。`sim_control.py` 和 `main_window.py` 的接入应一次性完成，实体就绪后无需再改；当前默认集结演示配置为 `configs/rally_demo_5_aircraft.json`。

### 11.1 sim_control — 角色映射与实体选择

`_NodeAlgorithm.__init__` 中当前仅识别 `"leader"` / 其他，需扩展为：

```python
if role == "leader":
    self._entity = LeaderEntity()
elif role == "rally_leader":
    self._entity = RallyLeaderEntity()
elif role == "rally_follower":
    self._entity = RallyFollowerEntity()
else:
    self._entity = FollowerEntity()
```

#### 11.1.1 禁用现有僚机预置逻辑

`_NodeAlgorithm.__init__` 中有一段"僚机预置"（line 754）：

```python
if role != "leader" and initial_leader_state is not None and hasattr(self._entity, "cxt"):
    self._entity.cxt.cmd.stage = FormStageE.HOLD   # ← 会命中 rally_follower
    ...
```

`rally_follower` 满足 `role != "leader"`，会被强制预置为 `cmd.stage=HOLD`，而集结僚机冷启动的正确初态是 `NONE`。需将条件改为：

```python
if role not in {"leader", "rally_leader", "rally_follower"} and initial_leader_state is not None and ...:
```

排除所有已知非 Hold 僚机角色（当前项目配置使用 `"wingman"` 而非 `"follower"`，用排除式避免遗漏）。

### 11.2 sim_control — 初始化参数差异

`RallyLeaderEntity.init` 额外需要：

- `cfg.route`：从 JSON 顶层统一 `route` 字段解析，前两点用于集结，完整航线用于后续任务飞行
- `cfg.rally_cfg`：从 JSON `rally_cfg` 字段解析为 `RallyTaskInitS`

`RallyFollowerEntity.init` 额外需要：

- `cfg.route`/`cfg.rally_cfg`：与长机共用同一份（M_i 由 `init()` 按 `route[0/1]`、`formation.slots`
  与 `rally_cfg.looseScale` 自动推导，**不再有逐节点 `rally_target` 配置字段**，见第八节 8.2/8.3）
- `cfg.rally_approach_speed_mps`：从 JSON `rally.approach_speed_mps` 读取
- `cfg.rally_leader_id`：从节点配置 `leader_id` 字段读取（`str`），传给 `FollowerBroadcastInitS.leaderId`

仿真层解析路径：`sim_control._init_modules_unlocked` 遍历节点 JSON（与 `_node_roles` 同循环），从每个节点 dict 读取 `node.get("leader_id", "")` 后传给 `_NodeAlgorithm.__init__` 的新参数 `node_config: dict`（或单独参数 `rally_leader_id: str`）；`_NodeAlgorithm.__init__` 再在构造 `EntityInitS` 时写入 `rally_leader_id=...`；`RallyFollowerEntity.init` 最终将其注入 `FollowerBroadcastInitS(leaderId=cfg.rally_leader_id)`。

`EntityInitS` 扩展字段已在第八节定义，仿真层直接填充即可。

### 11.3 sim_control — remote 与 now_s 注入

当前 `_NodeAlgorithm.step`（line 759）签名无 `remote` 参数，内部硬编码 `RemoteCmdS(FormStageE.HOLD)`；`EntityInputS` 也没有 `now_s`。需同时扩展：

**`_NodeAlgorithm.step` 新签名**：

```python
def step(
    self,
    state: AircraftState,
    inbox: list[MessageEnvelope],
    time_s: float,
    remote: RemoteCmdS,          # 新增：由控制器统一传入
    health: str = "normal",
) -> _NodeAlgorithmOutput:
```

内部改为：

```python
self._entity.step(
    EntityInputS(
        selfState=_motion_from_aircraft_state(state),
        inbox=inbox,
        remote=remote,           # 不再硬编码 HOLD
        now_s=time_s,
    ),
    entity_output,
)
```

**控制器侧 `_remote_stage` 变量**：

`SimulationController` 新增字段：

```python
self._remote_stage: FormStageE = FormStageE.HOLD  # 默认 Hold；load_config() 按节点角色自动切为 RALLY
```

`_run_formation_algorithms_unlocked` 调用 step 时传入：

```python
output = self._node_algorithms[node_id].step(
    state, inbox, self._time_s,
    remote=RemoteCmdS(self._remote_stage),
    health=health_map.get(node_id, "normal"),
)
```

**唯一方案：控制器按角色自动管理 `_remote_stage`**，调用方无需手动设置：

- `load_config()` 时：若节点列表中存在 `rally_leader` 或 `rally_follower` 角色，自动设 `_remote_stage = RALLY`；否则设 `HOLD`（Hold 场景默认值不变）。同时清空 `_formation_completed_analysis`。
- `reset()` 时：重新按角色检测，规则同上；清空 `_formation_completed_analysis`。
- 算法循环末尾：检测到 `_formation_completed_analysis` 刚被锁存（从 None 变为非 None）时，自动将 `_remote_stage` 切为 `HOLD`。
- `_remote_stage` 不对外暴露接口；集结是否完成由算法状态自动推进，不依赖 GUI 判断。

**首拍集结预热**：`run_until_complete()` 会在首个 tick 前自动开始集结，GUI/API 也可能在 `start()` 后立即调用 `start_rally()`；此时 `RallyJoinPos` 尚未执行 STANDBY，待命圆仍为空。控制器在 `tick_index == 0` 且集结节点仍为 `LOCAL_LOITER` 运行模式时，先调用一次 `_run_formation_algorithms_unlocked()` 建立各机待命圆，再切换 `ACTIVE/RALLY`。该预热不推进通信时钟、模型或仿真时间；已经运行过 tick 或重复调用开始集结时不再执行。

### 11.4 sim_control — formationAnalysis 传递链路

`entity_output.formationAnalysis` 从实体层到仿真快照需经过两步传递：

**第一步：`_NodeAlgorithm.step` → `_NodeAlgorithmOutput`**

`_NodeAlgorithmOutput`（或等价结构体）新增字段：

```python
formation_analysis: FormationAnalysisS | None = None
```

`_NodeAlgorithm.step` 在 `return` 时一并构造（不用临时赋值，与现有 `_NodeAlgorithmOutput` 构造风格一致）：

```python
return _NodeAlgorithmOutput(
    control=...,
    control_diag=...,
    outbox=...,
    status=...,
    formation_analysis=entity_output.formationAnalysis,  # 非首帧为 None
)
```

**第二步：`SimulationController` 聚合 → `SimulationSnapshot`**

`_run_formation_algorithms_unlocked` 里的 `output` 是局部变量，`_make_snapshot_unlocked` 无法访问 `node_outputs`。在 `SimulationController` 上新增**持久**字段，使实体侧的一次性结果能够保留在后续控制器快照中：

```python
self._formation_completed_analysis: FormationAnalysisS | None = None
```

**清空时机**（以下两种情况需清空，防止旧结果污染新一轮集结）：

- `load_config()`：重新加载配置时清空
- `reset()`：复位仿真时清空（`_remote_stage` 由控制器自动管理，不存在"设回 NONE"路径，NONE 语义仅在实体/单元层可见）

在算法循环末尾更新（仅在收到首帧时锁存，已锁存则不覆盖）：

```python
for node_id, state in states.items():
    output = self._node_algorithms[node_id].step(...)
    ...
    if output.formation_analysis is not None and self._formation_completed_analysis is None:
        self._formation_completed_analysis = output.formation_analysis  # 仅锁存首帧
        self._remote_stage = FormStageE.HOLD                            # 完成后自动切 HOLD（见 11.3 节）
```

`SimulationSnapshot` 新增字段（构造时传入，`frozen=True` 不可事后赋值）：

```python
rally_analysis: FormationAnalysisS | None = None
```

`_make_snapshot_unlocked` 构造时传入锁存值：

```python
return SimulationSnapshot(
    ...,
    rally_analysis=self._formation_completed_analysis,
)
```

分析结果的传递链路止于 `SimulationSnapshot.rally_analysis`。当前 `ControllerSimulationAdapter` 和 GUI `Snapshot` 不包含该字段，界面也不展示编队分析结果；后续若增加界面展示，应另行设计并补充对应实现与测试。

### 11.5 GUI — 演示场景切换按钮

> 界面层打桩，不涉及算法。集结实体尚未实现时，集结按钮加载桩配置即可；实体就绪后无需修改 UI。

编队保持和集结是两个互斥演示场景。在 `src/ui/gui/main_window.py` → `_build_left_panel()` 最顶部新增"演示场景"分组（位于"配置"分组之上）：

```text
┌─ 演示场景 ──────────────┐
│  [  编队保持  ]          │
│  [  集结演示  ]          │
└──────────────────────────┘
```

```python
# _build_left_panel 中，config_group 前插入
demo_group = QGroupBox("演示场景")
demo_layout = QVBoxLayout(demo_group)
demo_layout.setContentsMargins(10, 18, 10, 10)
demo_layout.setSpacing(6)
hold_btn = QPushButton("编队保持")
rally_btn = QPushButton("集结演示")
hold_btn.clicked.connect(lambda: self._load_demo("hold"))
rally_btn.clicked.connect(lambda: self._load_demo("rally"))
demo_layout.addWidget(hold_btn)
demo_layout.addWidget(rally_btn)
layout.addWidget(demo_group)
```

新增方法：

```python
def _load_demo(self, kind: str) -> None:
    root = default_project_root()
    config_files = {
        "hold":  root / "configs" / "base.json",
        "rally": root / "configs" / "rally_demo_5_aircraft.json",
    }
    path = config_files.get(kind)
    if path is None or not path.exists():
        self._log("WARN", f"演示配置文件不存在: {path}")
        return
    self._apply_config_path(str(path))
```

### 11.6 默认集结演示配置 `configs/rally_demo_5_aircraft.json`

**此文件已作为当前默认集结演示配置保留在仓库**（`configs/rally_demo_5_aircraft.json`）。

`rally_demo_5_aircraft.json` 角色使用 `"rally_leader"` / `"rally_follower"`，节点 ID 为 `A01/A02/A03/A04/A05`，A03 为集结长机，初始坐标分散放置以模拟集结前离散态。文件复用五机队形文件，可正常加载、运行集结流程并切换队形。

实体就绪后需完整替换为正式配置：

1. 将角色改为 `"rally_leader"` / `"rally_follower"` 并补全集结专属字段（参见第八节）。
2. 配置统一 `route_file`，使其成为初始有效航线；采用避障规划结果后，以覆盖航线的 `route[0]` 作为集结中心、`route[0]→route[1]` 作为集结和出航方向，后续航点组成任务航线。
3. 槽位偏置旋转方向使用统一航线第一航段方向角 θ，无需维护额外的一致性约束。

届时 `_remote_stage` 会自动切为 RALLY，无需修改 GUI 代码。

LLT 对此文件的验证范围：文件存在且能被 `sim_control.load_config()` 解析，路径为 `configs/rally_demo_5_aircraft.json`；旧三机 `configs/rally_demo.json` 不再作为仓库演示配置保留。
