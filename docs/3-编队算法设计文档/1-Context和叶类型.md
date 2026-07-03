# 一、叶类型

叶类型本质是一堆结构体（dataclass）、枚举的定义

## 1.1 枚举

```python
from enum import IntEnum

class FormStageE(IntEnum):   # 编队阶段（指令/状态共用）
    NONE = 0
    RALLY = 1                # 集结
    HOLD = 2                 # 编队保持
    RECONFIG = 3             # 编队重构

# 注：编队队形不再用枚举表示。cmd.pattern 是"纯整型队形索引"（0 起），
# 直接作为 formPos 的行号；队形语义由配置的队形列表顺序决定，代码不认死任何队形名。

class CommDirE(IntEnum):     # 通信方向
    DUPLEX = 0               # 双向
    SIMPLEX = 1              # 单向
```



## 1.2 结构体

> 其中M是队形个数，N是飞机个数

坐标命名约定：

- 东北天坐标系字段使用 `east/north/h` 表示位置，使用 `east/north/up` 表示速度和加速度。
- 航迹坐标系字段使用 `x/y/z`，其中 `x` 为前向，`y` 为垂向，`z` 为右侧向；该约定主要用于控制诊断量和航迹系中间量。
- `FormPosS.x/y/z` 是队形槽位相对长机的局部坐标：`x` 为沿长机航迹前向，`y` 为垂向/上向，`z` 为右侧向。它与控制诊断中的航迹坐标系 `x/y/z` 保持同一轴序，但不表示东北天坐标。
- 配置文件显式声明 `formation.slots` 时必须同时声明 `formation.coordinate_system = "x_forward_y_up_z_right"`，避免轴序不一致的槽位配置被静默解释。

```python
from dataclasses import dataclass, field

# ===================== 初始信息 =====================
@dataclass
class FormSelfInitS:                       # 各个飞机的独有信息
    id: str = ""                           # 飞机ID（节点ID，如 "A01"，与通信模块节点 ID 一致）

@dataclass
class NetWorkS:                            # 网络拓扑中的一条通信链路
    startId: str = ""                      # 链路起点飞机ID（节点ID）
    endId: str = ""                        # 链路终点飞机ID（节点ID）
    dir: CommDirE = CommDirE.DUPLEX        # 该链路的通信方向

@dataclass
class FormPosS:                            # 队形中单个槽位的位置
    id: str = ""                           # 飞机ID（节点ID），SlotGeometry 按它匹配查槽位
    x: float = 0.0                         # 队形槽位局部前向偏移，单位：m
    y: float = 0.0                         # 队形槽位垂向/上向偏移，单位：m
    z: float = 0.0                         # 队形槽位右侧向偏移，单位：m

@dataclass
class FormCommInitS:                       # 共有初始信息（集中式通信下发）
    netWork: list[NetWorkS] = field(default_factory=list)        # 网络拓扑（多条链路）
    formPat: list[str] = field(default_factory=list)             # [M] M种队形的名字（仅供显示，索引=队形号）
    formPos: list[list[FormPosS]] = field(default_factory=list)  # [M][N] 每种队形下各机槽位位置；cmd.pattern 直接取第几行
    initialPattern: int = 0                                      # 初始队形索引（cmd.pattern 初值）

# ===================== 和单机飞行/控制诊断相关 =====================
@dataclass
class PosInEarthS:                         # 在地球平面坐标系下的位置
    east: float = 0.0                      # 单位：m
    north: float = 0.0                     # 单位：m
    h: float = 0.0                         # 单位：m

@dataclass
class VdInEarthS:                          # 在地球平面坐标系下的地速
    vEast: float = 0.0                     # 东向速度分量
    vNorth: float = 0.0                    # 北向速度分量
    vUp: float = 0.0                       # 天向速度分量
    vTheta: float = 0.0                    # 航迹倾角
    vPsi: float = 0.0                      # 航迹偏航角
    vd: float = 0.0                        # 地速大小，=sqrt(vEast**2 + vNorth**2)
    dVPsi: float = 0.0                     # 航迹偏航角速率（水平航向角变化率，rad/s），用于转弯向心前馈

@dataclass
class AccInEarthS:                         # 在地球平面坐标系下的加速度
    accEast: float = 0.0                   # 东向加速度
    accNorth: float = 0.0                  # 北向加速度
    accUp: float = 0.0                     # 天向加速度

@dataclass
class MotionProfS:                         # 运动剖面（位置+速度），指令/状态共用
    pos: PosInEarthS = field(default_factory=PosInEarthS)   # 位置
    v: VdInEarthS = field(default_factory=VdInEarthS)       # 地速

@dataclass
class PosTrackDiagS:                       # 位置跟踪诊断量，不进 Context
    cmd_pos_east_m: float = 0.0            # 目标位置，东北天 east
    cmd_pos_north_m: float = 0.0           # 目标位置，东北天 north
    cmd_pos_h_m: float = 0.0               # 目标位置，东北天 h
    cmd_vel_east_mps: float = 0.0          # 目标速度，东北天 east
    cmd_vel_north_mps: float = 0.0         # 目标速度，东北天 north
    cmd_vel_up_mps: float = 0.0            # 目标速度，东北天 up
    pos_err_east_m: float = 0.0            # 位置误差，东北天 east
    pos_err_north_m: float = 0.0           # 位置误差，东北天 north
    pos_err_h_m: float = 0.0               # 位置误差，东北天 h
    vel_err_east_mps: float = 0.0          # 速度误差，东北天 east
    vel_err_north_mps: float = 0.0         # 速度误差，东北天 north
    vel_err_up_mps: float = 0.0            # 速度误差，东北天 up
    track_pos_err_x_m: float = 0.0         # 位置误差，航迹系 x 前向
    track_pos_err_y_m: float = 0.0         # 位置误差，航迹系 y 垂向
    track_pos_err_z_m: float = 0.0         # 位置误差，航迹系 z 右侧向
    track_vel_err_x_mps: float = 0.0       # 速度误差，航迹系 x 前向
    track_vel_err_y_mps: float = 0.0       # 速度误差，航迹系 y 垂向
    track_vel_err_z_mps: float = 0.0       # 速度误差，航迹系 z 右侧向

# ===================== 和航线相关 =====================
@dataclass
class WayPointS:                           # 内部航路点，携带该点起始段的属性
    idx: int = 0                           # 航路点编号，从0开始
    pos: PosInEarthS = field(default_factory=PosInEarthS)   # 航路点位置
    vdCmd: float = 0.0                     # 该点起始段的地速指令，m/s
    turnSign: float = 0.0                  # 该点起始段转向：+1 左转/逆时针、-1 右转/顺时针、0 直线
    center: PosInEarthS = field(default_factory=PosInEarthS) # 圆弧圆心(仅 turnSign!=0 有意义)

@dataclass
class WayLineS:                            # 单段航段；start.* 描述本段属性，end.* 描述下一段属性
    idx: int = 0                           # 航段编号
    start: WayPointS = field(default_factory=WayPointS)     # 起点；start.turnSign!=0 表示本段为圆弧
    end: WayPointS = field(default_factory=WayPointS)       # 终点；end.turnSign/vdCmd 描述下一段

# 辅助：圆弧半径 = hypot(start.pos.east - start.center.east, start.pos.north - start.center.north)
# RouteS 已删除；内部用 list[WayLineS]，外部输入用 list[WayPointInputS]

@dataclass
class WayPointInputS:                      # 用户/A* 输入的原始航点，由 leader.init() 转换为 WayLineS 序列
    idx: int = 0                           # 航点编号，从0开始
    pos: PosInEarthS = field(default_factory=PosInEarthS)   # 航点位置
    vdCmd: float = 0.0                     # 该点起始段的地速指令，m/s
    r: float = 0.0                         # 拐点处的期望转弯半径(米)；0=不插圆弧；首末点无意义
    turnSign: float = 0.0                  # 已知圆弧时填入；0 表示直线或待按 r 计算
    center: PosInEarthS = field(default_factory=PosInEarthS) # 圆弧圆心(turnSign!=0 时有意义)

# ===================== 和编队相关 =====================
@dataclass
class FormSnapshotS:                       # 编队快照
    stage: FormStageE = FormStageE.NONE    # 编队阶段
    pattern: int = 0                       # 当前队形索引（formPos 行号，0 起）；语义由配置队形列表顺序决定
    step: int = 0                          # 阶段内步骤

@dataclass
class RemoteCmdS:                          # 遥控指令（外部输入，由对象组边界持有，不进 Context；单枚举包一层以满足端口绑定粒度）
    stage: FormStageE = FormStageE.NONE    # 遥控下发的编队阶段
```



# 二、context

> 只有需要**跨拍保留**或**被多个单元读写**的工作状态才放入 Context。据此，只被单个单元使用的边界 I/O（`inbox`/`outbox` 的 `list[MessageEnvelope]`、`remote` 遥控指令）、初始化配置（`FormCommInitS`/`FormSelfInitS`/`RouteS`）和只供日志/UI 观察的诊断量（如 `PosTrackDiagS`）都不放入 Context，由对象组在边界持有或汇总到 `EntityOutputS`。收到的 envelope 解析后，只将其中的数据写入 Context（长机运动状态写入 `leaderState`，模态与队形写入 `cmd`）。完整规则见《0-HLD.md》3.3。`MessageEnvelope` 由通信模块定义，见《5-1-通信功能LLD.md》§4.1（`src/common/envelope.py`），不属于本文叶类型。

```python
@dataclass
class FormContextS:                        # 编队算法上下文（跨拍常驻、原地迭代）
    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)      # 编队指令（模态+队形）
    state: list[FormSnapshotS] = field(default_factory=list)      # [N] 各机编队状态（N包含所有飞机）
    wayLine: WayLineS = field(default_factory=WayLineS)           # 当前跟踪的航段；整条航线 RouteS 不进 Context
    nextWayLine: WayLineS = field(default_factory=WayLineS)       # 下一航段（供曲率前馈跨段前瞻；末段时同当前段）
    leaderState: MotionProfS = field(default_factory=MotionProfS) # 长机运动状态（僚机用，来自长机广播解析）
    selfCmd: MotionProfS = field(default_factory=MotionProfS)     # 本机目标运动状态（位置解算输出）
    selfState: MotionProfS = field(default_factory=MotionProfS)   # 本机当前运动状态（传感器）
    selfAccCmd: AccInEarthS = field(default_factory=AccInEarthS)  # 跟踪输出的三轴加速度指令
```
