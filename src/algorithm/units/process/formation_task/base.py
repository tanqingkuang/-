"""编队任务编排基础接口。注意：具体任务需写入任务输出端口。"""

from __future__ import annotations

from dataclasses import dataclass

from src.algorithm.context.leaf_types import FormSnapshotS, RemoteCmdS


@dataclass
class FormationTaskInitS:
    pass


@dataclass
class FormationTaskInputS:
    remote: RemoteCmdS | None = None
    cmd: FormSnapshotS | None = None


@dataclass
class FormationTaskOutputS:
    cmd: FormSnapshotS | None = None


class FormationTaskBase:
    def init(self, cfg: FormationTaskInitS) -> None:
        """按配置初始化 FormationTaskBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: FormationTaskInputS, y: FormationTaskOutputS) -> None:
        """推进 FormationTaskBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 FormationTaskBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError
