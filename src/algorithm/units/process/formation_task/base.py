"""编队任务编排基础接口。注意：具体任务需写入任务输出端口。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, RemoteCmdS


@dataclass
class FormationTaskInitS:
    """编队任务初始化配置基类。注意：当前无字段，预留派生扩展。"""

    pass


@dataclass
class FormationTaskInputS:
    """编队任务输入端口。注意：remote 为外部期望，cmd 为当前快照。"""

    remote: RemoteCmdS | None = None  # 外部遥控指令
    cmd: FormSnapshotS | None = None  # 当前编队指令快照（可作输入参考）


@dataclass
class FormationTaskOutputS:
    """编队任务输出端口。注意：cmd 承载任务决策出的阶段与队形。"""

    cmd: FormSnapshotS | None = None  # 输出的编队指令


class FormationTaskBase:
    """编队任务编排抽象基类。注意：子类须把决策出的阶段/队形写入 y.cmd。"""

    def init(self, cfg: FormationTaskInitS) -> None:
        """按配置初始化 FormationTaskBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: FormationTaskInputS, y: FormationTaskOutputS) -> None:
        """推进 FormationTaskBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 FormationTaskBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
