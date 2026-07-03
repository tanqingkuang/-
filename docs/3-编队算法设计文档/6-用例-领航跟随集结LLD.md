# 领航跟随集结 LLD

> 对应场景：多机分散位置 → 集结航线集结 → 队形保持（mission_route）

---

## 一、说明

本文档描述领航跟随集结场景（`entity/leader_follower_rally/`）的低层设计，供人阅读，也用于指导代码开发。

本场景在领航跟随保持（`leader_follower_hold/`）基础上新增集结能力，完全遵循《0-HLD.md》架构原则：**不修改现有实体实现和既有单元实现**；允许扩展公共叶类型（`leaf_types.py`）、Context（`context.py`）及实体边界类型（`EntityInputS`/`EntityInitS`/`EntityOutputS`）。新建实体放在新目录，复用/扩展所需的单元族。

**关于实体代码复用**：`RallyLeaderEntity` 与现有 `LeaderEntity` 有大量结构相似的代码（位置解算、跟踪、输出回填等）。当前版本选择**直接新建完整实体**，原因是：① Hold 场景不受影响，无需回归测试；② FormationTask（Hold vs Rally）是二选一，无法在同一实例中兼容；③ 功能验证优先，过早提取基类会增加当前实现风险。待集结功能稳定、所有场景装配方案确定后，可一次性提取 `LeaderEntityBase` / `FollowerEntityBase` 消除重复，改动集中、风险可控。

---

## 二、总体策略与状态机

### 2.1 设计原则

**集结期间三机平等**：JOINING 阶段（cmd.step=0）不区分长机/僚机，所有飞机用同一套 `RallyJoinPos` 算法飞向各自的预设松散目标点 M_i。只有当全部飞机完成集结（均进入 EXITED 状态）后，才切换到 CATCHUP 阶段，此时 R01 开始作为编队参考源生成槽位目标，R02/R03 进入跟随模式。

**CATCHUP 阶段过渡**：全员切出时各机在沿航迹方向的位置是分散的——最晚到达 M_i 的飞机直接切出后已向前飞出一段距离，而早到的飞机（包括掌机）刚刚从盘旋圆切出，位置可能落后。CATCHUP 阶段通过速度调制消除这一散布，使三机相对间距收敛到松散队形要求，之后才进入二维槽位跟随（LOOSE）。

**长机角色切换时机**：CATCHUP 开始后掌机切换到任务航线（`mission_route`）飞行，僚机以掌机当前位置为参考动态计算各自槽位目标。LOOSE/COMPRESS/HOLD 阶段行为与领航跟随保持场景完全相同。

### 2.2 四阶段集结策略

---

#### 第一步：集结航线与松散目标点（初始化时自动计算）

**集结航线定义两个关键点**：

- **A = `rally_route[0]`**：集结区起点，掌机在 JOINING 阶段的目标位置，也是松散队形的中心，**也应该是
  `mission_route` 的起点**（详见"第二步"末尾"航线连续性约束"）
- **B = `rally_route[-1]`**：集结航线文件的最后一个航点。当前实现只用到 `rally_route[0]`（A）和
  `rally_route[1]`（A1，仅用于推导航向），**B 不参与任何计算**——早期版本文档曾把 B 当作"集结区终点/
  `mission_route` 起点"，那是重构前"松散队形中心=rally_route 终点"旧设计遗留的错误描述，本次"M_i 自动
  计算"重构已把中心改成 A（起点），正确的连续性约束是 `mission_route` 起点应等于 A，不是 B

```text
A  = rally_route[0]  集结区起点，也应是 mission_route 起点
A1 = rally_route[1]  第一航段终点（用于推导航向）
B  = rally_route[-1] 集结航线文件末端航点（不参与计算，仅供人工设计航线时参考）

θ = atan2(A1_north − A_north, A1_east − A_east)  # 第一航段方向角，初始化时自动推导
R(θ) = [[cos θ, −sin θ], [sin θ, cos θ]]          # 将体坐标偏置旋转到 ENU

掌机松散目标：M_leader = A（飞向集结区起点）
僚机松散目标：M_i = A + R(θ) × (looseScale × slot_i_ENU)
             高度：A.h + slot.y（固定差，不随 looseScale 扩展）
```

各机的松散目标点由仿真层在配置加载时自动计算，**无需在配置文件中逐机写死**。slot 坐标系 x_forward_y_up_z_right 到 ENU 的映射：east = slot_x，north = −slot_z（z_right = 南向）。

三机示例（A=(0, 5000)，θ=0°（正东），looseScale=3）：

| 节点       | 队形槽位 (x_m, z_m) | slot→ENU (east, north) | × looseScale   | M_i (east, north) |
| ---------- | ------------------- | ---------------------- | -------------- | ----------------- |
| R01（长机）| (0, 0)              | (0, 0)                 | (0, 0)         | (0, 5000) = A     |
| R02        | (-54, -58)          | (-54, +58)             | (-162, +174)   | (-162, 5174)      |
| R03        | (-54, +58)          | (-54, -58)             | (-162, -174)   | (-162, 4826)      |

---

#### 第二步：JOINING 阶段（切线进圆 + 盘旋协调汇合）

每架飞机独立运行 `RallyJoinPos` 单元，经历三个内部状态：

**盘旋圆几何（init 时按任务航向定死，不随到达方向变化）**：

```python
C = M_i + R × (−sin θ_m, cos θ_m)   # 圆心，θ_m = mission_heading_rad，R 向任务航向左侧偏置
θ_slot = atan2(M_i.north − C.north, M_i.east − C.east)  # M_i 在圆上的固定角度
```

这样摆放保证 M_i 处的 CCW 切线方向恒等于任务航向 θ_m，与飞机从哪个方向飞来无关——这是本节相对旧版最核心的改动：旧版用"到达 M_i 时的速度方向"摆圆心，导致盘旋圆朝向和实际到达方向绑定，一旦到达方向和任务航向差异较大（例如飞机从任务航线下游一侧飞向集结点），切出瞬间指令会相对盘旋指令发生大角度跳变，表现为"切出后先反向飞一段再掉头"。改为按任务航向定死圆心后，不管飞机从哪个方向飞来，只要沿圆弧飞到 M_i 就必然对齐任务航向，跳变问题不再出现。

**FLYING**：直飞盘旋圆的 CCW 切入点 T，每帧计算并广播 ETA（直飞 T 的时间 + 沿圆弧从 T 到 M_i 的估算时间）。

- T 只在 FLYING 第一拍按当前位置算一次后固定，此后不再重算（避免目标漂移）。
- T 的求法：把当前位置看作圆外一点，对盘旋圆作两条切线，取其中"直飞方向在切点处能顺势接上 CCW 切向"的那一条切点——两条切线里另一条接的是 CW，方向不对，不能用。
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
- **已知限制**：起点已经落在盘旋圆内部/圆上（无切线可求）时，退化为直飞 M_i（等价于旧版行为），这种场景暂未细化，留待后续按需补齐。
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
  - `ang_dist > _SLOT_ANG_AWAY (≈60°)` → 标记"已远离"；`ang_dist < _SLOT_ANG_NEAR (≈20°)` + 已远离 → 触发评估
  - 刚从切入点 T 进弧时，只有 **T 到 M_i 的真实 CCW 弧长本身也很小**（< `_SLOT_ANG_NEAR`）才直接置位
    "已远离"，允许首次路过 M_i 就评估切出（"最后到达"飞机的场景）；弧长不小则必须按标准流程先飞过
    "远离"窗口、再等真正接近 M_i 时才评估。
    > `ang_dist` 是对称弧距，分不清"T 恰好在 M_i 之前一点"（真快到了，弧长小）和"T 恰好在 M_i 之后
    > 一点"（弦长虽近，CCW 方向其实还要绕近一整圈才能到）。如果进弧就无条件置位"已远离"（不看真实弧长），
    > 后一种情况会被误判成"已到达"，在没有真正沿圆弧飞完的情况下就直接切出——原样复现了这次重构本想
    > 根除的"切出瞬间指令跳变"问题。用真实弧长而非对称弧距来决定初始"已远离"状态即可避免。
- 每次经过 M_i 时评估切出：
  - `remaining = T_ref − t_now`
  - 若 `remaining < 2πR/v_max / 2`：**立即切出** → EXITED
  - 否则：调整速度使本圈周期 ≈ remaining，再飞一圈（`_away_from_slot` 复位）

**EXITED**：从 M_i 位置沿任务航向（`mission_heading_rad`）直飞，交由 CATCHUP 接管；切出瞬间的指令方向恒等于任务航向，不再依赖飞机是"先到"还是"最后到"。

> **一致性**：切入点 T 到 M_i 的首次路过，与盘旋过程中每次经过 M_i，使用同一套角度检测和同一贪心逻辑（`_should_exit`），因此"最后到机"只是"首次路过 M_i 恰好满足切出条件"的特例，不需要单独分支。

**T_ref 计算**（长机 Rally 任务，每帧）：

```python
# FLYING 全部计入；LOITERING 只有尚未首次路过 M_i 时才计入（reachedSlotOnce=False）。
flying_etas = [
    s.eta_s for s in followerStates
    if s.rally_state != "EXITED" and (s.rally_state == "FLYING" or not s.reachedSlotOnce)
]
leader_counts = not leader_join_exited and (leader_join_flying or not leader_join_reached_slot_once)
if leader_counts:
    flying_etas.append(leader_eta_s)
T_ref = max(flying_etas) if flying_etas else t_now
t_ref_valid = leader_ready and all_expected_followers_ready
```

> **设计要点（`reachedSlotOnce`，本次"切线进圆"重构新增）**：切入点 T 到 M_i 之间可能还有很长一段弧
> 要飞（见 FLYING 小节），"进入 LOITERING"不再等价于"已经到过 M_i"，因此不能像旧版那样简单地用
> `rally_state=="FLYING"` 判断是否计入 T_ref——那样会导致刚到切入点、实际还要飞一大段弧才第一次到
> M_i 的飞机被过早剔除，T_ref 塌缩到别的（更快）参与者的 ETA，造成同步提前。修复方式：`RallyJoinPos`
> 新增 `reached_slot_once` 状态（进圆后只要几何上真正路过 M_i 附近一次就置位，不依赖 `t_ref_valid`），
> 通过 `formation.follower_status` 广播（`reached_slot_once` 字段）传给长机，写入
> `FollowerStateS.reachedSlotOnce`；长机自身同理经 `leader_join_reached_slot_once` 注入 Rally 任务。
> T_ref 聚合规则变为：**FLYING 一律计入；LOITERING 只有尚未首次路过 M_i 时才计入**——已经路过至少一次、
> 纯粹在等 T_ref 的飞机不再计入，否则它每圈波动的"下次路过还要多久"会反复推高/拉低 T_ref。

`ready` 表示参与者已经实际执行过至少一拍汇合解算：FLYING 状态必须携带大于当前时刻的有限 ETA，LOITERING / EXITED 状态直接视为已初始化。长机尚未收齐全部期望参与者的首个有效状态时，仍可计算诊断用 T_ref，但必须广播 `t_ref_valid=False`；早到机此时以最低盘旋速度等待，不得使用默认 `T_ref=t_now` 切出。旧格式广播没有 `t_ref_valid` 字段时同样按 False 处理。

T_ref 与 `t_ref_valid` 通过长机广播（`RallyLeaderBroadcast`）下发给各机；仅当 `t_ref_valid=True` 时，各机才据此调整盘旋速度并执行切出判定。

**通信链路**（见图 4）：

- 僚机 → 长机：`formation.follower_status` 消息，含 `{pos, eta_s, rally_state, reached_slot_once, arrived}`
- 长机 → 僚机：`formation.leader` 消息，含 `{cmd, slot_scale, t_ref, t_ref_valid, leader_state}`

**JOINING → CATCHUP 门控**（`_all_participants_exited`）：

1. 长机自身 `rally_state == EXITED`
2. 所有 `expectedFollowerIds` 机均 `rally_state == EXITED`
3. 以上状态均在 `stale_timeout_s` 内有效

> **说明**：各机 EXITED 的时间先后不同——最后到达 M_i 的飞机直接切出（不盘旋），而先到的飞机需等到 T_ref ≈ t_now 时才在盘旋圆上的下一次经过 M_i 时切出。因此全员切出瞬间，各机在沿航迹方向的位置是分散的（早切出的飞机已飞出一段距离），需要 CATCHUP 阶段来收敛相对间距。

#### 已知限制与待办

以下是 JOINING/`RallyJoinPos` 当前设计已确认的取舍或遗留缺口，记录以便后续排期，均不是本次改动引入的新缺陷：

1. **起点落在盘旋圆内部/圆上时暂无切线求法**（见"FLYING"小节）：退化为直飞 M_i，等价于本次重构前的旧行为，尚未验证这种场景下的表现是否可接受。触发条件：飞机初始位置到集结点 M_i 的距离 ≤ `loiter_radius_m`。
2. **进场角度不利的"迟到"飞机可能要多绕近一整圈才能切出**：切入点 T 到 M_i 的真实 CCW 弧长完全由飞机相对盘旋圆的进场方向决定；如果这个弧长恰好接近 360°，即使 T_ref 已经要求"现在就该切出"，飞机也必须先飞完这段弧才能以正确航向到达 M_i 切出——这是保证"切出航向恒等于任务航向"这个几何约束的必然代价，不是能单靠调参消除的问题。极端情况下会让单机集结耗时明显变长；如果后续场景对集结总时长敏感，需要评估是否要在集结点/进场方向的选取上做额外约束来规避大弧长进场。
3. **`loiter_radius_m` 有一个由 `approach_speed_mps`/`control_period_s` 反推出的隐式下限**（见 `RallyJoinPos.init()`）：半径太小时，切入圆弧的触发半径会被地板值或离散步进距离压过 5° 航向跳变角上限，init 会直接拒绝。这个下限不是配置里能直接看到的一个数字，而是每次 init 时按当前 `approach_speed_mps`/`control_period_s` 现算的，调这两个参数时要留意联动影响。

---

#### 第三步：CATCHUP 阶段（追赶对齐）

**触发条件**：全员 EXITED，Rally 任务从 step=0 切换到 step=1。

**目标**：各机沿任务航向直线飞行。横侧向修正到各自专属"杆"，前向通道通过速度调制追赶目标槽位点。

##### "杆"模型

每架飞机有一条专属**杆（rod）**：过本机 M_i 点、平行于任务航向的直线。杆的横侧坐标在 `CatchupAlign.init` 时一次性计算并固定：

```python
_mi_cross = −M_i.east × sin_h + M_i.north × cos_h   # 初始化后不再变化
```

##### 真实槽位（语义目标）

```text
slot_from_leader = 掌机当前位置 + scale × rotate(slot_offset, 任务航向)
slot_along       = slot_from_leader · heading_unit       # 沿航迹分量，随掌机移动
true_slot        = (slot_along, _mi_cross)               # 投影到杆上
```

真实槽位在 GUI 中显示为 diamond，位于掌机后方正确位置（`CatchupAlign.true_slot_east / true_slot_north`）。

##### 两个独立控制通道

**① 横侧向（位置控制器）**：

位置控制器目标设为**本机在杆上的正交投影**（而非真实槽位）：

```python
selfCmd.pos = (self_along, _mi_cross)  # 前向误差=0，位置PID只产生横向修正
```

> **为何用投影而非真实槽位**：若 `selfCmd.pos = true_slot` 且飞机超前槽位，位置 PID 会产生反向（后退）加速度，与速度调制叠加导致飞机掉头。使用投影点（前向误差=0），位置 PID 仅修正横向偏差，沿航迹完全由速度调制承担，两通道解耦。

**② 前向（速度调制）**：

```text
along_track_err = slot_along − self_along
speed = clamp(v_nominal + kp_speed × along_track_err, v_min, v_max)
```

其中 `kp_speed` 在当前实现中固定为 `0.05 m/s per m`，不从场景配置读取。

- 超前槽位（`along_track_err < 0`）→ 降速等待，始终向前飞
- 落后槽位（`along_track_err > 0`）→ 提速追赶

```python
selfCmd.v.vEast  = speed × cos_h
selfCmd.v.vNorth = speed × sin_h
selfCmd.v.vPsi   = mission_heading
selfCmd.v.vd     = speed
selfCmd.v.dVPsi  = 0.0
```

##### CATCHUP → LOOSE 门控

```text
posErr_m = hypot(along_track_err, _mi_cross − self_cross)
```

Rally 任务检查：所有期望僚机同时满足 **`posErr_m < catchup_radius_m`（默认 200 m）** 和 **航向误差 `< 0.17 rad`**，并连续保持 3 秒后切换到 LOOSE。航向阈值和连续保持时间是当前实现的内部固定值，不从场景配置读取。

---

#### 第四步：LOOSE → COMPRESS → HOLD（松散收紧）

长机沿 `mission_route` 飞，僚机跟随 `ScaledSlotGeometry` 槽位。

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
| 2          | LOOSE    | 松散间距二维槽位跟随，等待收敛                 |
| 3          | COMPRESS | 线性压缩至最终间距                             |

### 2.4 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `loiter_radius_m` | 200 m | 盘旋圆半径 |
| `arrival_radius_m` | 100 m | 触发到达判断的距离阈值 |
| `slot_hit_radius_m` | 60 m | 判断"飞经 M_i"的距离阈值 |
| `mission_heading_rad` | — | 切出后飞行方向，由 A→A1（集结航线第一航段方向）自动推导，不在配置中写死 |
| `loiter_speed_min/max` | 14/25 m/s | 盘旋速度上下限（固定翼约束） |
| `catchup_radius_m` | 200 m | CATCHUP→LOOSE 门控阈值（二维槽位距离，米） |
| `last_arrival_threshold_s` | 5 s | 兼容保留，当前不参与切出判定 |

到达时是否跳过盘旋不使用独立配置参数，而是固定按 `remaining < t_loop_min / 2` 判断；其中 `t_loop_min = 2π × loiter_radius_m / loiter_speed_max_mps`。配置入口仍保留 `last_arrival_threshold_s`，用于兼容现有场景文件和后续策略扩展；当前版本读取并透传该值，但不会用它改变切出时机。

---

## 三、新增叶类型

以下类型新增到 `src/algorithm/context/leaf_types.py`。

### 3.1 `RallySlotScaleS` — 槽位缩放因子

```python
@dataclass
class RallySlotScaleS:
    """集结阶段的槽位偏置缩放因子。scale=1.0 为最终队形，>1.0 为松散放大。
    注意：需要跨拍保留，且被 FormationTask/Rally 写、PosCalc/ScaledSlotGeometry 读，故进 Context。"""
    scale: float = 1.0
    scaleRate: float = 0.0   # scale 对时间的导数（1/s）；LOOSE 为 0，COMPRESS 为负值
    # ScaledSlotGeometry 用 scaleRate 计算因压缩产生的额外速度前馈，避免在单元内存储上一拍 scale
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
    eta_s: float = 0.0           # 预计到达松散点的仿真时刻（秒）；LOITERING/EXITED 时为当前时刻
    rally_state: str = "FLYING"  # 集结汇合状态：FLYING / LOITERING / EXITED
    reachedSlotOnce: bool = False  # 是否已至少一次路过 M_i，供 T_ref 聚合判断是否仍需被等待
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

在 `FormContextS` 中新增两个集结专用字段（进 Context 的条件：跨拍保留 + 多单元读写）：

```python
@dataclass
class FormContextS:
    # ... 已有字段 ...
    slotScale: RallySlotScaleS = field(default_factory=RallySlotScaleS)
    # 被 FormationTask/Rally(写) 与 PosCalc/ScaledSlotGeometry(读)

    followerStates: list[FollowerStateS] = field(default_factory=list)
    # 被 Inbound/FollowerStatus(写) 与 FormationTask/Rally(读)
    # 注意：list 在移植 C 时改为定长数组+计数器
```

`reset_context` 同步扩展：`slotScale.scale = 1.0, slotScale.scaleRate = 0.0`，`followerStates.clear()`。

---

## 五、新增流程组单元

### 5.1 FormationTask/Rally — 集结任务编排

**文件**：`units/process/formation_task/rally.py`

作用：管理 APPROACH→LOOSE→COMPRESS→HOLD 状态机，写出 `cmd`（stage/step）和 `slotScale`。

#### 5.1.1 抽象类扩展

`FormationTaskInitS` 与 `FormationTaskInputS`/`FormationTaskOutputS` 基类不变；`Rally` 子类扩展输入/输出端口结构体：

```python
@dataclass
class RallyTaskInitS(FormationTaskInitS):
    looseScale: float = 3.0               # 松散槽位放大倍数（松散间距=最终间距×looseScale）
    convergenceRadius_m: float = 5.0      # 到达判定阈值，米
    arriveHold_s: float = 3.0             # APPROACH→LOOSE 需持续在阈值内的时间
    stableHold_s: float = 5.0             # LOOSE→COMPRESS 需稳定的时间
    compressTime_s: float = 30.0          # COMPRESS 阶段持续时间（scale 从 looseScale→1.0）
    tightRadius_m: float = 2.0            # COMPRESS→HOLD 精度阈值，米
    expectedFollowerIds: list[str] = field(default_factory=list)
    # 期望参与集结的僚机 ID 列表；all(arrived) 只在此列表全部满足时成立；空列表→立即通过（测试用）
    staleTimeout_s: float = 2.0           # 超过此时长未收到某机报文则视为数据失效
    targetPattern: int = 0
    # 集结只用单队形（formPos 第 0 行），进入 LOOSE/COMPRESS 时 cmd.pattern 写入此索引，供 ScaledSlotGeometry 查槽位
    dt_s: float = 0.02                    # 控制周期（秒）；进 InitS 才能在 init 时校验 > 0

@dataclass
class RallyTaskInputS(FormationTaskInputS):
    # 继承 remote: RemoteCmdS, cmd: FormSnapshotS
    followerStates: list[FollowerStateS] = None  # 端口 → Context.followerStates
    now_s: float = 0.0    # 当前仿真时间（秒），由实体从边界输入注入，用于超时判断

@dataclass
class RallyTaskOutputS(FormationTaskOutputS):
    # 继承 cmd: FormSnapshotS
    slotScale: RallySlotScaleS = None       # 端口 → Context.slotScale
    rallyCompleted: bool = False            # COMPRESS→HOLD 正常完成时置 True，仅该拍有效；实体据此输出 FormationAnalysisS
    t_ref: float = 0.0                      # 本拍计算的最晚 ETA
    t_ref_valid: bool = False               # 是否已收齐所有参与者首个有效汇合状态
```

#### 5.1.2 Rally 子类实现逻辑

**`init`**：存储配置参数，初始化内部计时器 `_arrive_timer`、`_stable_timer`、`_compress_elapsed`。参数合法性断言（`dt_s` 在 InitS 中，init 时即可校验，违反则抛出异常）：`looseScale >= 1.0`、`compressTime_s > 0`、`staleTimeout_s > 0`、`dt_s > 0`。

**`step`** 顶层逻辑（先处理 remote，再按 cmd.step 路由）：

> **每拍开头先置 `y.rallyCompleted = False`**，再进入 remote/step 路由。OutputS 对象可能被复用，不显式置 False 则上一拍的 True 会泄漏到后续帧。
>
> `u.remote` 由实体缓存为 `self._remote`（默认 `RemoteCmdS(stage=NONE)`），仅在外部 `EntityInputS.remote` 非 None 时更新，故 Rally 单元侧收到的 `u.remote` 始终非 None，无需空值保护。

```text
remote == NONE:
  若 cmd.stage in {RALLY, HOLD}:       # 正在集结或已完成，收到 NONE 复位
    reset 所有计时器
    实体同步 reset RallyJoinPos；长机清除 _rally_completed 和上一轮 followerStates
  输出 cmd.stage=NONE, cmd.step=0, cmd.pattern=NONE, slotScale.scale=looseScale, scaleRate=0
  return

remote == HOLD:
  若 cmd.stage == RALLY:               # 外部强制切 HOLD（中断集结）
    reset 所有计时器
  输出 cmd.stage=HOLD, cmd.step=0, cmd.pattern=targetPattern, slotScale.scale=1.0, scaleRate=0
  return

remote == RALLY:
  若 cmd.stage == HOLD:                # 集结已完成，HOLD 是终态；忽略 RALLY 重启
    输出 cmd.stage=HOLD, cmd.step=0, cmd.pattern=targetPattern,
         slotScale.scale=1.0, slotScale.scaleRate=0
    return                             # 只有先发 NONE 再发 RALLY 才能重新集结
  若 cmd.stage == NONE:               # 首次进入集结
    reset 所有计时器
    cmd.step = 0
  # cmd.stage == RALLY → 继续集结，按 cmd.step 路由
```

**`step`** 按 `u.cmd.step` 路由（只在 remote.stage == RALLY 且 cmd.stage != HOLD 时执行）：

```text
辅助函数 is_valid(entry):
  entry 未找到 OR valid==False OR (now_s - lastUpdate_s) > staleTimeout_s → False；否则 True

辅助函数 all_followers_arrived():
  （用于 APPROACH→LOOSE：检查每机是否已到达 M_i，依赖僚机锁存的 arrived 标志）
  若 expectedFollowerIds 为空 → True（测试用）
  对每个 id: is_valid(entry)==False OR arrived!=1 → False
  全部通过 → True

辅助函数 all_followers_ok(threshold_m):
  （用于 LOOSE→COMPRESS 和 COMPRESS→HOLD：检查槽位误差收敛）
  若 expectedFollowerIds 为空 → True
  对每个 id: is_valid(entry)==False OR posErr_m >= threshold_m → False
  全部通过 → True

（说明：各子阶段先计算 next_step/next_stage，本拍统一输出新值，避免输出与内部状态矛盾）

sub=APPROACH:
  检查 all_followers_arrived()
    是 → _arrive_timer += dt_s；若达到 arriveHold_s → next_step=1（LOOSE），_arrive_timer=0
    否 → _arrive_timer = 0；next_step=0
  输出 cmd.stage=RALLY, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=looseScale, slotScale.scaleRate=0
  （注：从第一拍起就写 cmd.pattern；next_step=1 时本拍输出已切换，僚机下拍进入 LOOSE）

sub=LOOSE:
  检查 all_followers_ok(convergenceRadius_m)（posErr_m 此时为到松散槽位的误差）
    是 → _stable_timer += dt_s；若达到 stableHold_s → next_step=2（COMPRESS），_stable_timer=0
    否 → _stable_timer = 0；next_step=1
  输出 cmd.stage=RALLY, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=looseScale, slotScale.scaleRate=0

sub=COMPRESS:
  _compress_elapsed += dt_s
  scale = looseScale - (looseScale-1.0) × (_compress_elapsed / compressTime_s)
  若 scale <= 1.0:
    scale = 1.0
    scaleRate = 0.0          # 已到终值，清零速率；避免负值前馈持续驱动 ScaledSlotGeometry
  否则:
    scaleRate = -(looseScale-1.0) / compressTime_s
  若 scale==1.0 且 all_followers_ok(tightRadius_m):
    next_stage=HOLD, next_step=0, y.rallyCompleted=True
  否则:
    next_stage=RALLY, next_step=2
  输出 cmd.stage=next_stage, cmd.step=next_step, cmd.pattern=targetPattern,
       slotScale.scale=scale, slotScale.scaleRate=scaleRate
```

**`reset`**：清零所有内部计时器，`cmd.stage=NONE, cmd.step=0, cmd.pattern=NONE`，`slotScale.scale=looseScale, slotScale.scaleRate=0`。

测试用例：

- expectedFollowerIds 为空 → 计时器立即累加（测试）
- 期望列表非空但 followerStates 为空 → all_followers_arrived=False，不切换
- 某机超时（断链）→ is_valid=False，计时器冻结
- 某机 arrived==0（未到 M_i）→ all_followers_arrived=False，_arrive_timer 不推进
- 某机 arrived==1 但槽位误差大 → APPROACH 照常推进（arrived 与 posErr 语义分离），LOOSE 阶段该机 posErr 不满足则 _stable_timer 重置
- COMPRESS 过程 scale 线性变化验证

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
    selfCmd: MotionProfS = None    # 端口 → Context.selfCmd，当前目标（用于计算 posErr_m）
    selfArrived: int = 0           # 实体锁存的到达标志（_self_arrived），单次集结过程中锁存；仅 reset() 清零

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
| `eta_s` | float | 预计到达松散点的仿真时刻，秒 |
| `reached_slot_once` | bool | 是否已至少一次路过 M_i，供长机 T_ref 聚合判断本机是否仍需被等待 |

实现：`posErr_m = \|selfState.pos - selfCmd.pos\|`；`arrived = u.selfArrived`；按上表打包为 envelope，topic=`formation.follower_status`，target 为 `cfg.leaderId`（init 时由配置显式传入，不依赖 netWork 推断角色）。

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

作用：在 `LeaderBroadcast` 基础上，额外将 `slotScale.scale` 和 `slotScale.scaleRate` 打入广播，让僚机知道当前缩放因子和压缩速率。

> `LeaderBroadcast` 已广播 `cmd.stage/pattern/step`，本子类在同一 envelope 的 payload 中追加 `slot_scale` 字段：

```python
payload = {
    "leader_state": _motion_payload(u.selfState),
    "cmd": {"stage": int(u.cmd.stage), "pattern": int(u.cmd.pattern), "step": int(u.cmd.step)},
    "slot_scale": {"scale": u.slotScale.scale, "scale_rate": u.slotScale.scaleRate},
}
```

InitS/OutputS 直接复用父类，无需新建：`RallyLeaderBroadcastInitS = OutboundInitS`；`RallyLeaderBroadcastOutputS = OutboundOutputS`（含 `outbox`）。

```python
@dataclass
class RallyLeaderBroadcastInputS(OutboundInputS):
    # 继承 cmd: FormSnapshotS, selfState: MotionProfS
    slotScale: RallySlotScaleS = None   # 端口 → Context.slotScale（含 scale + scaleRate）
    t_ref: float = 0.0
    t_ref_valid: bool = False
```

---

### 5.5 Inbound/RallyLeaderFollower — 集结僚机解析长机广播

**文件**：`units/process/inbound/rally_leader_follower.py`

作用：在 `LeaderFollower` 基础上额外解析 `slot_scale`，写入 `Context.slotScale`。

```python
@dataclass
class RallyLeaderFollowerOutputS(InboundOutputS):
    # 继承 leaderState: MotionProfS, cmd: FormSnapshotS
    slotScale: RallySlotScaleS = None   # 端口 → Context.slotScale
    t_ref: float = 0.0
    t_ref_valid: bool = False           # 缺字段或非法值时保持 False
```

**多消息胜出规则**：同帧 inbox 中可能有多条 `formation.leader` 消息（重发或乱序）。遍历 inbox 时**按序处理，每条完整有效消息均覆盖写入** `leaderState/cmd/slotScale`，故最后一条有效消息最终胜出。`leaderState/cmd/slotScale` 三个字段必须来自**同一条消息**，不允许跨消息拼装，避免字段不一致。

解析逻辑（完整防御，任何异常均 fallback 到默认值）：

```python
for envelope in inbox:
    if envelope.topic != "formation.leader":
        continue
    payload = _parse_envelope(envelope)
    if payload is None:
        continue
    # 先写 leaderState + cmd（复用 LeaderFollower 父类逻辑）
    _write_leader_state_and_cmd(payload, y)
    # 再追加 slot_scale（同一消息，三字段一致性有保证）
    try:
        ss = payload.get("slot_scale", {})
        if not isinstance(ss, dict):
            raise TypeError
        y.slotScale.scale     = float(ss.get("scale",      1.0))
        y.slotScale.scaleRate = float(ss.get("scale_rate", 0.0))
    except (TypeError, ValueError):
        y.slotScale.scale     = 1.0
        y.slotScale.scaleRate = 0.0
```

三种需要捕获的情况：① `payload` 中无 `slot_scale` 键（旧版消息兼容）；② `slot_scale` 不是 dict；③ 字段值为非数字字符串（`float()` 抛 `ValueError`）。

---

## 六、新增算法组单元

### 6.1 PosCalc/RallyApproach — 飞向目标集结点

**文件**：`units/algo/pos_calc/rally_approach.py`

作用：APPROACH 子阶段专用，输出"直飞预分配目标点 M_i"的目标运动状态。不感知 Mode，纯计算。

```python
@dataclass
class RallyApproachInitS(PosCalcInitS):
    target: PosInEarthS = field(default_factory=PosInEarthS)  # 本机目标集结点 M_i
    approachSpeed_mps: float = 20.0   # 飞向目标时的水平地速
    k_alt: float = 0.5                # 近零水平距离时的高度比例增益（1/s）
    vUpMax_mps: float = 5.0           # 高度环输出限幅（m/s）

@dataclass
class RallyApproachInputS(PosCalcInputS):
    # 继承 selfState: MotionProfS → Context.selfState
    pass

# 输出复用 PosCalcOutputS（selfCmd → Context.selfCmd）
```

实现（令 `dN = target.north - self.north`，`dE = target.east - self.east`，`dH = target.h - self.h`，`dHoriz = sqrt(dN²+dE²)`，近零阈值 `ε_horiz = 0.5 m`）：

`vd` 是水平地速，水平和垂向独立计算；近零水平距离单独处理：

```text
selfCmd.pos = target

若 dHoriz >= ε_horiz:                        # 正常飞向目标
  vPsi   = atan2(dN, dE)
  vNorth = approachSpeed_mps × dN / dHoriz
  vEast  = approachSpeed_mps × dE / dHoriz
  vUp    = clamp(approachSpeed_mps × dH / dHoriz, -vUpMax_mps, vUpMax_mps)
  # 水平距离接近 ε_horiz 时 dH/dHoriz 可能很大，须限幅
  vd     = approachSpeed_mps               # 水平地速指令幅值

否则（dHoriz < ε_horiz，水平上已到目标正上/正下方）:
  保持当前航向不变（vPsi = selfState.v.vPsi）
  vNorth = 0, vEast = 0
  vd     = 0.0                             # 水平速度为零，与 hypot(vNorth,vEast) 一致
  vUp    = clamp(dH × k_alt, -vUpMax_mps, vUpMax_mps)   # 纯高度环，参数来自 RallyApproachInitS
```

测试：`dHoriz >= ε_horiz` → hypot(vNorth, vEast) ≈ approachSpeed，vUp 被限制在 vUpMax_mps 内；`dHoriz < ε_horiz` → vNorth=vEast=0，vd=0，vUp 被限制在 vUpMax_mps 内；两分支 vUp 均不超限。

---

### 6.2 PosCalc/ScaledSlotGeometry — 带缩放的槽位几何

**文件**：`units/algo/pos_calc/scaled_slot_geometry.py`

作用：在 `SlotGeometry` 基础上，读入 `slotScale.scale`，将槽位偏置乘以 scale 后再计算目标位置。LOOSE 和 COMPRESS 子阶段均使用此单元；COMPRESS 阶段 scale 随时间线性减小，目标位置随之平滑收敛。

```python
@dataclass
class ScaledSlotInitS(PosCalcInitS):
    selfId: str = ""
    commInit: FormCommInitS = field(default_factory=FormCommInitS)

@dataclass
class ScaledSlotInputS(SlotGeometryInputS):
    # 继承 selfState: MotionProfS → Context.selfState
    # 继承 leaderState: MotionProfS → Context.leaderState（来自 SlotGeometryInputS）
    # 继承 cmd: FormSnapshotS → Context.cmd（来自 SlotGeometryInputS）
    slotScale: RallySlotScaleS = None  # 端口 → Context.slotScale（新增字段）
    # 继承 SlotGeometryInputS 而非 PosCalcInputS，确保 super().step() 类型兼容

# 输出复用 PosCalcOutputS（selfCmd → Context.selfCmd），无需新增 OutputS 类
```

`ScaledSlotGeometry.init()` 实现要点：`SlotGeometry.init()` 需要 `selfId/formPat/formPos`，必须从 `ScaledSlotInitS.commInit` 手动组装 `SlotGeometryInitS` 再调用 `super().init()`：

```python
def init(self, cfg: ScaledSlotInitS) -> None:
    super().init(SlotGeometryInitS(
        selfId=cfg.selfId,
        formPat=cfg.commInit.formPat,
        formPos=cfg.commInit.formPos,
    ))
    # 父类 _form_pat / _form_pos 在此时才完成初始化；遗漏此调用则 step() 必抛 ValueError
```

**实现方式**：直接继承 `SlotGeometry`，在 `super().step()` 返回结果上做后处理，无需重复槽位查找逻辑。

```python
class ScaledSlotGeometry(SlotGeometry):
    def step(self, u: ScaledSlotInputS, y: PosCalcOutputS) -> None:
        super().step(u, y)                          # 先按 scale=1 算标准槽位
        scale     = u.slotScale.scale
        scaleRate = u.slotScale.scaleRate

        # 世界坐标系下的未缩放偏置（super 已算好）
        offset_e = y.selfCmd.pos.east  - u.leaderState.pos.east
        offset_n = y.selfCmd.pos.north - u.leaderState.pos.north
        offset_h = y.selfCmd.pos.h     - u.leaderState.pos.h

        # 位置缩放
        y.selfCmd.pos.east  = u.leaderState.pos.east  + scale * offset_e
        y.selfCmd.pos.north = u.leaderState.pos.north + scale * offset_n
        y.selfCmd.pos.h     = u.leaderState.pos.h     + scale * offset_h

        # 速度：d/dt(scale·R·slot) = scale·dR/dt·slot + scaleRate·R·slot
        # super 给出 leaderVel + dR/dt·slot；提取旋转前馈再乘 scale，加 scaleRate 项
        ff_e  = y.selfCmd.v.vEast  - u.leaderState.v.vEast
        ff_n  = y.selfCmd.v.vNorth - u.leaderState.v.vNorth
        ff_up = y.selfCmd.v.vUp    - u.leaderState.v.vUp    # 高度前馈（super 未缩放）
        y.selfCmd.v.vEast  = u.leaderState.v.vEast  + scale * ff_e  + scaleRate * offset_e
        y.selfCmd.v.vNorth = u.leaderState.v.vNorth + scale * ff_n  + scaleRate * offset_n
        y.selfCmd.v.vUp    = u.leaderState.v.vUp    + scale * ff_up + scaleRate * offset_h
        y.selfCmd.v.vd     = hypot(y.selfCmd.v.vEast, y.selfCmd.v.vNorth)
        y.selfCmd.v.vPsi   = atan2(y.selfCmd.v.vNorth, y.selfCmd.v.vEast)
        # dVPsi（偏航角速率）不随 scale 变化，保持父类值不动
```

`scale=1.0` 且 `scaleRate=0` 时后处理退化为精确复现 `SlotGeometry`（偏置无放大、无速度修正）；LOOSE 阶段 `scale=looseScale>1, scaleRate=0`，位置偏置放大但无额外速度项；COMPRESS 期间 `scaleRate<0` 自动添加向内的速度前馈。

测试：

- scale=1.0 → pos/v 结果与现有 SlotGeometry 相同
- scale=2.0 → 位置偏置扩大一倍，速度前馈同步缩放
- scale 从 2.0 线性减到 1.0 → 目标位置平滑收敛，vEast/vNorth/vUp 同步变化

---

### 6.3 PosCalc/CatchupAlign — 追赶对齐

**文件**：`units/algo/pos_calc/catchup_align.py`

作用：CATCHUP 子阶段（step=1）专用。两通道独立控制：横侧向修正到目标航线，前向速度调制收敛沿航迹槽位误差。复用 `ScaledSlotGeometry` 计算动态槽位目标。

```python
@dataclass
class CatchupAlignInitS:
    selfId:              str            = ""
    commInit:            FormCommInitS  = field(default_factory=FormCommInitS)
    mission_heading_rad: float          = 0.0   # 任务航向（弧度，东向为 0）
    nominal_speed_mps:   float          = 20.0  # 额定速度（与掌机任务航线速度一致）
    kp_speed:            float          = 0.05  # 沿航迹误差增益（m/s per m）
    speed_min_mps:       float          = 14.0  # 速度下限
    speed_max_mps:       float          = 25.0  # 速度上限
```

`step(u: ScaledSlotInputS, y: PosCalcOutputS)` 实现逻辑：

```python
# 1. 计算动态槽位目标（随掌机移动）
slot_geom.step(u, y)          # y.selfCmd.pos = slot_target

# 2. 沿航迹误差
along_track_err = (slot_target - self_pos) · heading_unit

# 3. 速度调制（前向通道）
speed = clamp(nominal + kp × along_track_err, v_min, v_max)

# 4. 速度指令（锁定航向，无横向修正）
selfCmd.v.vEast  = speed × cos_h
selfCmd.v.vNorth = speed × sin_h
selfCmd.v.vPsi   = mission_heading
selfCmd.v.vd     = speed
selfCmd.v.dVPsi  = 0.0

# 5. selfCmd.pos 保持 slot_target
#    横侧向：位置控制器用 pos.north 将飞机修正到目标航线
#    前向：kpPos=0，pos.east 不驱动前向运动，槽位在身后也不产生反向修正
```

`posErr_m` 由 `FollowerBroadcast` 自动计算为 `dist3d(selfState, selfCmd) ≈ dist2d(self, slot_target)`，直接用于 CATCHUP→LOOSE 门控，无需额外接口。

测试：

- 落后槽位（along_track_err > 0）→ speed > nominal，最大不超过 speed_max
- 超前槽位（along_track_err < 0）→ speed < nominal，最小不低于 speed_min
- 航向始终锁定 mission_heading，vPsi 不随槽位方向变化

---

## 七、新增实体

### 7.1 RallyLeaderEntity

**文件**：`entity/leader_follower_rally/leader.py`

#### 7.1.1 使用的单元子类

> 本节曾描述"两个 `LeaderRoute`（rally_route + mission_route）+ RouteInterp"贯穿 RALLY 全程的旧架构，
> 那是 `RallyJoinPos`（切线进圆汇合）出现之前的设计，已和当前实现不符——JOINING 阶段（step=0）现在完全
> 由 `RallyJoinPos` 负责位置解算，不经过 TraPlan/RouteInterp；`LeaderRoute`+`RouteInterp` 只在 CATCHUP
> 及之后（step>=1）用于沿 `cfg.route`（mission_route）飞行。以下按实际实现更正。

| 单元 | 子类 |
| --- | --- |
| 收消息 Inbound | FollowerStatus（解析僚机回报） |
| 任务编排 FormationTask | Rally（集结状态机） |
| 汇合位置解算（仅 JOINING，step=0） | RallyJoinPos（切入盘旋圆→圆弧盘旋→切出，见第二步） |
| 轨迹规划 TraPlan（仅 CATCHUP 及之后，step>=1 / HOLD） | LeaderRoute（`cfg.route`，即 mission_route） |
| 位置解算 PosCalc（仅 CATCHUP 及之后） | RouteInterp（复用现有） |
| 跟踪 PosTrack | PidCompose（复用现有） |
| 发消息 Outbound | RallyLeaderBroadcast（扩展，含 slotScale） |

#### 7.1.2 调用顺序（一拍 step）

```text
收消息(FollowerStatus)                    ← 解析僚机回报 → Context.followerStates
→ 任务编排(Rally)                         ← 读 followerStates/remote → 写 cmd + slotScale
→ 按 cmd.step 分流：
    JOINING（step=0）：      汇合位置解算(RallyJoinPos)     ← 直飞切入点 T/盘旋/切出，不经 TraPlan/RouteInterp
    CATCHUP 及之后（step>=1）/HOLD：轨迹规划(LeaderRoute) → 位置解算(RouteInterp)  ← 沿 mission_route 飞行
→ 跟踪(PidCompose)                        ← 复用，两条分支共用
→ 发消息(RallyLeaderBroadcast)            ← 广播 selfState + cmd + slotScale
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
    # RALLY step>=1（CATCHUP/LOOSE/COMPRESS）或 HOLD：长机沿任务航线（mission_route）飞行
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

**集结航线关键点（A / A1 / B）**：

- `A = rally_route[0].pos`：集结区起点，掌机的 JOINING 目标位置，也是松散队形中心，**也应该是
  `mission_route` 起点**（不是 B，见下方"航线连续性约束"）
- `A1 = rally_route[1].pos`：集结航线第一航段终点，仅用于推导航向（使多航点集结航线也能取到一个确定的航向）
- `B = rally_route[-1].pos`：集结航线文件末端航点，**当前实现不使用它做任何计算**（早期版本文档误将其
  当作"mission_route 起点"，是重构前"松散队形中心=rally_route 终点"旧设计的遗留错误描述，已在此更正）

掌机的 `RallyJoinPosInitS.loose_slot` 应设为 A；`mission_heading_rad` 取**第一航段方向**（A→A1），而非整条集结航线的 A→B 方向：

```python
A  = cfg.rally_route[0].pos   # cfg.rally_route: list[WayPointInputS]
A1 = cfg.rally_route[1].pos   # 第一航段终点
mission_heading_rad = math.atan2(A1.north - A.north, A1.east - A.east)
```

> 集结航线只有两个航点（A、B 直连，当前 demo 场景即如此）时 A1 == B，上式与"A→B 方向"等价；一旦集结航线扩展为三个及以上航点，实现按**第一航段方向**定队形朝向和松散槽位旋转角，不会取整条航线的整体方向——设计多航点集结航线时需注意这一点，避免与本节公式理解偏差。

**航线连续性约束（`_ConfigLoader.validate` 运行时校验，配置加载阶段）**：`mission_route` 起点应等于
**A**（不是 B——`rally_route[-1]` 在代码里没有任何一处被引用，全局搜索为空），且集结区第一航段方向
（A→A1）应与任务航线出航方向保持一致，确保 RALLY→CATCHUP 切换时 `RouteInterp` 目标位置不跳变、槽位
偏置旋转轴与 CATCHUP 对准轴与任务飞行方向一致。

为什么是 A 不是 B：长机 JOINING 阶段收敛的位置就在 A（`loose_slot=A`）；切出（EXITED）后沿任务航向继续
直飞，直到全员切出才切到 CATCHUP/`RouteInterp`——这段等待期间长机已经沿 A→A1 方向飞出去一截。若
`mission_route` 从 A 以外的另一点起飞（比如 B），`RouteInterp` 的直线投影参数会被钳在 `t>=0`，长机的
实际位置若还没追上这个"起点"就会被拉回去等待，或者已经飞过头就会跳变；`mission_route` 从 A 起飞、且
第一段方向与任务航向一致时，长机的投影位置天然连续，不需要额外处理。

**`RallyLeaderEntity.init()` 本身仍只校验 `rally_route` 至少含两个航点，不做连续性检查**（历史版本曾有
`dist3d(rally_end, mission_start) < 1.0` 的 init 期校验，已随本次"M_i 自动计算"重构移除）——连续性校验
现在移到了更早的配置加载层：`sim_control_modules.py::_ConfigLoader.validate()` 在构造任何实体之前就检查
两条：

1. **位置**：`dist3d(route[0], rally_route[0])` 必须小于 1.0m；
2. **方向**：`route` 首段方向（`route[0]→route[1]`）与 `rally_route` 首段方向（A→A1，即
   `mission_heading_rad`）夹角必须小于 `_MAX_MISSION_RALLY_HEADING_MISMATCH_DEG`（10°）——只查位置
   不够，位置对得上但方向差很多（比如垂直、相反）时，JOINING(EXITED，沿 rally 方向飞)→CATCHUP(沿
   mission_route 方向飞) 切换瞬间仍会有真实的指令航向突变（实测垂直配置跳变 ~90°、相反配置 ~180°）。

任一条不满足都会在 `load_config()`/`validate()` 阶段直接报错，不再是"只在文档里约定、代码不检查"。
设计 `rally_route_file` 和 `route_file` 时最简单的做法仍是让两者直接复用同一份航线文件（`route[0]`
和 `rally_route[0]` 自然是同一个航点，方向也自然一致；当前 `configs/rally_demo.json` 即采用此做法）。

---

### 7.2 RallyFollowerEntity

**文件**：`entity/leader_follower_rally/follower.py`

#### 7.2.1 使用的单元子类

| 单元 | 子类 |
| --- | --- |
| 收消息 Inbound | RallyLeaderFollower（扩展，含 slotScale 解析） |
| 任务编排 FormationTask | 不使用（模态来自长机广播） |
| 轨迹规划 TraPlan | Noop（复用） |
| 位置解算 PosCalc | RallyJoinPos（JOINING）/ CatchupAlign（CATCHUP）/ ScaledSlotGeometry（LOOSE+COMPRESS） |
| 跟踪 PosTrack | PidCompose（复用） |
| 发消息 Outbound | FollowerBroadcast（回报位置与状态） |

#### 7.2.2 调用顺序（一拍 step）

```text
收消息(RallyLeaderFollower)               ← 解析长机广播 → leaderState + cmd + slotScale
→ [轨迹规划(Noop) 空策略]
→ 位置解算（按 cmd.stage + cmd.step 路由）
    cmd.stage==NONE:                        跳过 PosCalc，直接输出零速保持当前位置（不触发跟踪）
    cmd.stage==RALLY, cmd.step==0:          RallyJoinPos     ← JOINING：飞向 M_i / 盘旋 / 切出
    cmd.stage==RALLY, cmd.step==1:          CatchupAlign     ← CATCHUP：锁航向速度调制
    cmd.stage==RALLY, cmd.step>=2 / HOLD:   ScaledSlotGeometry ← LOOSE/COMPRESS/HOLD：二维槽位跟随
→ 跟踪(PidCompose)（NONE 时跳过）
→ 发消息(FollowerBroadcast)               ← 回报位置 + posErr + arrived
```

PosCalc 切换逻辑（L1 职责）：

```python
stage = self.cxt.cmd.stage

if stage == FormStageE.NONE:
    # 集结未开始，输出当前位置零速，不驱动跟踪
    copy_position(self.cxt.selfState.pos, self.cxt.selfCmd.pos)  # 逐字段复制，避免别名
    zero_velocity(self.cxt.selfCmd.v)
    zero_acceleration(self.cxt.selfAccCmd)  # 清加速度，防止上一帧残留
    # 仍执行 FollowerBroadcast，发送 arrived=0（让长机知道本机在线但尚未集结）
    self._follower_broadcast_u.selfArrived = 0
    self._outbound.step(self._outbound_u, self._outbound_y)
    # EntityOutputS 回填，与现有 hold 实体风格一致：
    # 调用方未提供容器则直接引用；否则逐字段写入，避免改变其对象引用
    if y.selfAccCmd is None:
        y.selfAccCmd = self.cxt.selfAccCmd
    else:
        y.selfAccCmd.accEast  = self.cxt.selfAccCmd.accEast   # 逐字段写入，避免改变引用
        y.selfAccCmd.accNorth = self.cxt.selfAccCmd.accNorth
        y.selfAccCmd.accUp    = self.cxt.selfAccCmd.accUp
    if y.selfCmd is None:
        y.selfCmd = self.cxt.selfCmd
    else:
        copy_motion(self.cxt.selfCmd, y.selfCmd)
    y.outbox.clear(); y.outbox.extend(self._outbound_y.outbox)
    return

elif stage == FormStageE.RALLY:
    # 本机自判到达：一旦到达 M_i 则锁存，不因误差回升而反转
    if not self._self_arrived:
        self._pos_calc_approach.step(self._approach_u, self._pos_calc_y)
        # 用 selfState.pos 与配置的 rally_target（M_i）比较，不用 selfCmd.pos
        # （RallyApproach 已把 selfCmd.pos 设为 target，比较二者距离永远为 0）
        dist = |self.cxt.selfState.pos - self._rally_target|  # 3D 距离
        if dist < self._arrive_threshold_m:
            self._self_arrived = True
    else:
        self._pos_calc_slot.step(self._slot_u, self._pos_calc_y)
    self._follower_broadcast_u.selfArrived = 1 if self._self_arrived else 0  # 每拍注入锁存值

else:  # HOLD：外部强制 HOLD，无论本机是否到达，均跟标准槽位
    self._pos_calc_slot.step(self._slot_u, self._pos_calc_y)
    self._follower_broadcast_u.selfArrived = 1 if self._self_arrived else 0  # 保持广播状态
```

> **说明**：`cmd.step` 是长机广播的系统级状态，反映"期望僚机是否全部到达"，而非驱动单机切换。单机到达即切换 PosCalc，让先到机以松散槽位跟随长机低速飞行，后到机仍在直飞追赶，两者可以并存于同一 APPROACH 系统阶段内。`cmd.step==2`（COMPRESS）开始时，`_self_arrived` 必已为 True，故本逻辑在 COMPRESS 阶段亦正确。

#### 7.2.3 初始化（init）关键点

- `_rally_target`：在 `init()` 中根据 `rally_route[0]`（A 点）、`rally_route[1]`（第一航段终点）及队形槽位本地计算（`A + R(θ) × (looseScale × slot_i_ENU)`），高度偏置固定为 `slot.y`（不随 `looseScale` 扩展），**不从配置字段读取**
- `_arrive_threshold_m`：从 `cfg.rally_cfg.convergenceRadius_m` 读入（与 Rally 单元保持一致）
- `_self_arrived`：初始化为 `False`，到达 M_i 后置 `True`，`reset()` 时归零
- 每拍 `step()` 中：`follower_broadcast_u.selfArrived = 1 if self._self_arrived else 0`（实体负责注入锁存值）
- **端口绑定（不可遗漏）**：
  - `RallyLeaderFollower` Inbound 的 `slotScale` 输出端口 → `Context.slotScale`
  - `ScaledSlotGeometry` 的 `leaderState` 端口 → `Context.leaderState`
  - `ScaledSlotGeometry` 的 `cmd` 端口 → `Context.cmd`
  - `ScaledSlotGeometry` 的 `slotScale` 端口 → `Context.slotScale`

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
    rally_route: list[WayPointInputS] = None  # 集结航线；至少两个航点；[0]=A（起点），[-1]=B（终点）
    rally_cfg: RallyTaskInitS = field(default_factory=RallyTaskInitS)  # 集结参数
    rally_approach_speed_mps: float = 20.0  # 僚机飞向 M_i 的速度
    rally_leader_id: str = ""               # 僚机回报消息的发送目标（来自节点配置 leader_id）
```

### 8.3 配置文件扩展（JSON）

```json
{
  "route_file": "element/rally_demo_route.json",
  "rally_route_file": "element/rally_demo_rally_route.json",
  "rally": {
    "loose_scale": 3.0,
    "convergence_radius_m": 5.0,
    "arrive_hold_s": 3.0,
    "stable_hold_s": 5.0,
    "compress_time_s": 30.0,
    "tight_radius_m": 2.0,
    "stale_timeout_s": 2.0,
    "expected_follower_ids": ["follower_1", "follower_2"],
    "approach_speed_mps": 20.0,
    "k_alt": 0.5,
    "v_up_max_mps": 5.0
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

> `route_file` 加载后展开为顶层 `route`（mission_route），`rally_route_file` 加载后展开为 `rally_route`。`route` 起点必须等于 `rally_route` 起点 A（不是终点 B——B 不参与任何计算）——此约束由航线设计保证（见 7.1.3 节），最简单的做法是将两者设为同一文件。
>
> **M_i 自动计算**：从机的松散目标点由实体 init 自动推导，无需在配置文件中逐机写死。实体读取 `rally_route[0]`（A）和 `rally_route[1]` 推导 θ，再结合 `formation.formation_files` 展开后的目标队形槽位和 `loose_scale` 计算 M_i。
>
> `k_alt` / `v_up_max_mps` 可省略，省略时使用 `RallyApproachInitS` 中的默认值（0.5/s，5.0 m/s）。

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
    │       ├── slot_geometry.py          ← 不动
    │       ├── rally_approach.py         ← 新建
    │       └── scaled_slot_geometry.py   ← 新建
    └── process/
        ├── formation_task/
        │   ├── hold.py                   ← 不动
        │   └── rally.py                  ← 新建
        ├── tra_plan/
        │   └── leader_route.py           ← 不动（两个实例在实体里管理）
        ├── outbound/
        │   ├── leader_broadcast.py       ← 不动
        │   ├── rally_leader_broadcast.py ← 新建
        │   └── follower_broadcast.py     ← 新建
        └── inbound/
            ├── leader_follower.py        ← 不动
            ├── rally_leader_follower.py  ← 新建
            └── follower_status.py        ← 新建
```

---

## 十、编队分析输出

仅当 COMPRESS 子阶段自然完成（内部 `_rally_completed` 标志置位）后的第一拍，`RallyLeaderEntity` 计算并输出 `FormationAnalysisS`。`remote==HOLD` 外部强制中断和 `remote==NONE` 复位均不触发分析输出。

`_rally_completed` 由 Rally 单元在 COMPRESS→HOLD 转换时写入输出（`y.rallyCompleted = True`），实体读取后置位本地标志；`remote==NONE` 时实体同步清除该标志（允许再次集结后重新输出）。

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

本节汇总 `sim_control.py`、`main_window.py` 以及 `configs/` 目录所需的改动。`sim_control.py` 和 `main_window.py` 的接入应一次性完成，实体就绪后无需再改；当前 `configs/rally_demo.json` 是临时桩配置，实体就绪后需替换为正式集结配置（见 11.6 节）。

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

- `cfg.rally_route`：从 JSON 顶层 `rally_route` 字段解析为 `RouteS`
- `cfg.rally_cfg`：从 JSON `rally` 字段解析为 `RallyTaskInitS`

`RallyFollowerEntity.init` 额外需要：

- `cfg.rally_target`：从节点配置 `rally_target` 字段解析为 `PosInEarthS`
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
        "rally": root / "configs" / "rally_demo.json",
    }
    path = config_files.get(kind)
    if path is None or not path.exists():
        self._log("WARN", f"演示配置文件不存在: {path}")
        return
    self._apply_config_path(str(path))
```

### 11.6 桩配置 `configs/rally_demo.json`

**此文件已作为本 LLD 的交付物新增到仓库**（`configs/rally_demo.json`，与 LLD 同步提交）。

`rally_demo.json` 角色使用 `"rally_leader"` / `"rally_follower"`，节点 ID 为 `A01/A02/A03`，初始坐标分散放置以模拟集结前离散态。文件复用三机三角队形文件，可正常加载并运行集结流程。

实体就绪后需完整替换为正式配置：

1. 将角色改为 `"rally_leader"` / `"rally_follower"` 并补全集结专属字段（参见第八节）。
2. 补充 `rally_route` 字段，并确保顶层 `route`（mission_route）的起点等于 `rally_route` 起点 A（不是终点 B——B 不参与任何计算）——**`_ConfigLoader.validate` 会在配置加载阶段做运行时校验**（见 7.1.3 节"航线连续性约束"），位置或方向不满足都会直接报错，不再只是文档约定。最简单的做法是将 `route_file` 与 `rally_route_file` 设为同一文件（`route[0]` 自然等于 `rally_route[0]`，方向也自然一致）。
3. 槽位偏置旋转方向（A→A1，即集结航线第一航段方向角 θ）须与任务航线方向一致，在设计 `rally_route` 时统一考虑——同样由第 2 条提到的运行时校验兜底。

届时 `_remote_stage` 会自动切为 RALLY，无需修改 GUI 代码。

LLT 对此文件的验证范围：文件存在且能被 `sim_control.load_config()` 解析，路径为 `configs/rally_demo.json`。
