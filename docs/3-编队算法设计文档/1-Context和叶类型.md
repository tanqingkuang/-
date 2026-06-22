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

class FormPatE(IntEnum):     # 编队队形
    NONE = 0
    TRIANGLE = 1             # 三角编队

class CommDirE(IntEnum):     # 通信方向
    DUPLEX = 0               # 双向
    SIMPLEX = 1              # 单向
```



## 1.2 结构体

> 其中M是队形个数，N是飞机个数

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
    x: float = 0.0                         # 相对队形基准的槽位位置，单位：m
    y: float = 0.0
    z: float = 0.0

@dataclass
class FormCommInitS:                       # 共有初始信息（集中式通信下发）
    netWork: list[NetWorkS] = field(default_factory=list)        # 网络拓扑（多条链路）
    formPat: list[FormPatE] = field(default_factory=list)        # [M] M种队形样式
    formPos: list[list[FormPosS]] = field(default_factory=list)  # [M][N] 每种队形下各机槽位位置

# ===================== 和单机飞行相关 =====================
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

@dataclass
class AccInEarthS:                         # 在地球平面坐标系下的加速度
    accEast: float = 0.0                   # 东向加速度
    accNorth: float = 0.0                  # 北向加速度
    accUp: float = 0.0                     # 天向加速度

@dataclass
class MotionProfS:                         # 运动剖面（位置+速度），指令/状态/误差共用
    pos: PosInEarthS = field(default_factory=PosInEarthS)   # 位置
    vd: VdInEarthS = field(default_factory=VdInEarthS)      # 地速

# ===================== 和航线相关 =====================
@dataclass
class WayPointS:                           # 航点
    idx: int = 0                           # 航点编号，从0开始
    pos: PosInEarthS = field(default_factory=PosInEarthS)   # 航点位置

@dataclass
class WayLineS:                            # 航段
    idx: int = 0                           # 航段编号，0-1的航点组成航段0
    start: WayPointS = field(default_factory=WayPointS)     # 起始航点
    end: WayPointS = field(default_factory=WayPointS)       # 终点航点
    vdCmd: float = 0.0                     # 航段速度指令，单位 m/s
    radius: float = 0.0                    # 曲率，0代表直线

@dataclass
class RouteS:                              # 航线
    lines: list[WayLineS] = field(default_factory=list)      # 多个航段

# ===================== 和编队相关 =====================
@dataclass
class FormSnapshotS:                       # 编队快照
    stage: FormStageE = FormStageE.NONE    # 编队阶段
    pattern: FormPatE = FormPatE.NONE      # 队形状态
    step: int = 0                          # 阶段内步骤

@dataclass
class RemoteCmdS:                          # 遥控指令（外部输入，由对象组边界持有，不进 Context；单枚举包一层以满足端口绑定粒度）
    stage: FormStageE = FormStageE.NONE    # 遥控下发的编队阶段
```



# 二、context

> 只有需要**跨拍保留**或**被多个单元读写**的工作状态才放入 Context。据此，只被单个单元使用的边界 I/O（`inbox`/`outbox` 的 `list[MessageEnvelope]`、`remote` 遥控指令）和初始化配置（`FormCommInitS`/`FormSelfInitS`/`RouteS`）都不放入 Context，由对象组在边界持有；收到的 envelope 解析后，只将其中的数据写入 Context（长机运动状态写入 `leaderState`，模态与队形写入 `cmd`）。完整规则见《0-HLD.md》3.3。`MessageEnvelope` 由通信模块定义，见《5-1-通信功能LLD.md》§4.1（`src/common/envelope.py`），不属于本文叶类型。

```python
@dataclass
class FormContextS:                        # 编队算法上下文（跨拍常驻、原地迭代）
    cmd: FormSnapshotS = field(default_factory=FormSnapshotS)      # 编队指令（模态+队形）
    state: list[FormSnapshotS] = field(default_factory=list)      # [N] 各机编队状态（N包含所有飞机）
    wayLine: WayLineS = field(default_factory=WayLineS)           # 当前跟踪的航段；整条航线 RouteS 不进 Context
    leaderState: MotionProfS = field(default_factory=MotionProfS) # 长机运动状态（僚机用，来自长机广播解析）
    selfCmd: MotionProfS = field(default_factory=MotionProfS)     # 本机目标运动状态（位置解算输出）
    selfState: MotionProfS = field(default_factory=MotionProfS)   # 本机当前运动状态（传感器）
    selfAccCmd: AccInEarthS = field(default_factory=AccInEarthS)  # 跟踪输出的三轴加速度指令
```
