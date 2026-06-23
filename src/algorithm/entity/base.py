"""编队实体基础接口。注意：具体实体需实现 init/step/reset/close。"""

from __future__ import annotations

from src.algorithm.entity.types import EntityInitS, EntityInputS, EntityOutputS


class EntityBase:
    def init(self, cfg: EntityInitS) -> None:
        """按配置初始化 EntityBase。注意：调用方需先准备好必要依赖和输入数据。"""
        raise NotImplementedError

    def step(self, u: EntityInputS, y: EntityOutputS) -> None:
        """推进 EntityBase 一个处理周期。注意：输入输出约定需与上下游模块保持一致。"""
        raise NotImplementedError

    def reset(self) -> None:
        """复位 EntityBase 的动态状态。注意：保留构造期依赖，只清理运行期数据。"""
        raise NotImplementedError

    def close(self) -> None:
        """释放 EntityBase 持有的资源。注意：关闭后不应继续调用运行接口。"""
        raise NotImplementedError
