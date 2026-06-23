"""轨迹规划基础接口。注意：输出航段需保持完整起终点信息。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, MotionProfS, WayLineS


@dataclass
class TraPlanInitS:
    """轨迹规划初始化配置基类。注意：具体规划器可派生扩展字段。"""

    pass


@dataclass
class TraPlanInputS:
    """轨迹规划输入端口。注意：selfState 用于按当前位置选择航段。"""

    cmd: FormSnapshotS | None = None  # 当前编队指令快照
    wayLine: WayLineS | None = None  # 上一帧航段（可作为输入参考）
    selfState: MotionProfS | None = None  # 本机运动状态


@dataclass
class TraPlanOutputS:
    """轨迹规划输出端口。注意：wayLine 为本帧选定的待跟踪航段。"""

    wayLine: WayLineS | None = None  # 输出的当前航段


class TraPlanBase:
    """轨迹规划单元抽象基类。注意：子类 step 须向 y.wayLine 写入完整起终点的航段。"""

    def init(self, cfg: TraPlanInitS) -> None:
        """按配置初始化 TraPlanBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: TraPlanInputS, y: TraPlanOutputS) -> None:
        """推进 TraPlanBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 TraPlanBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
