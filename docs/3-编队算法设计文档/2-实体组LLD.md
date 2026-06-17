# 实体组 LLD

> 实体组是编队算法中**唯一持有状态**的模块：每个实体 = 一个对象，实例化并组合它需要的算法库 / 流程库单元，持有这些子对象即持有全部维护数据。
> 接口契约见 `1-LLD综述.md`；本册讲实现方法。

## 1. 实体类型

| 实体 | `entity_type` | 组成 | 产 control |
| --- | --- | --- | --- |
| 飞机本体 | `flight` | 飞行单元；长机额外寄宿协调单元 | 是 |
| 协调本体（将来：地面站 / 虚拟节点 / 参考节点） | `coordination` | 仅协调单元，不飞 | 否 |

> 本轮只有飞机本体：长机本体（飞行 + 协调）、僚机本体 ×N（飞行）。

## 2. 挂接 / 实例化

- `init(entity_id, config)`：按 `config` **实例化所需的库单元类并组合进实体**（挂接 = 把单元对象作为实体的成员）；静态参数（增益、队形几何表、槽位号、`entity_type`）在此注入各单元的构造函数。
- **挂接 = 为每个族选一个实现**：每个单元都是一个策略族（抽象基类 + 多实现，见 `1-LLD综述.md` §2.2）；实体挂接时**给每个族挑定一个实现类并实例化**（如位置解算选"航线插值"还是"槽位几何"、编排选"常量保持"还是"状态机"、发选"广播"还是"不发"）。**"由实体来选实现"就落在这一步**——情景差异靠选不同实现，不靠单元内 `if/else`。
- **可重入靠实例化**：N 个僚机 = `Wingman` 类的 N 个实例，各持各的状态；库类定义是那份共享代码，禁止全局 / 类级可变态。
- **C 移植**：一个实体/单元对象 ≡ C 的 `struct + 接收 struct 指针的函数`；挂接 ≡ 工厂创建结构体、所有权归实体。

## 3. `step` 编排（本轮：显式接线 / 静态 DAG）

`step()` 内按**固定拓扑序**调用各单元，**为每个单元显式组装 `u`**（取自 `ctx` 字段与一个或多个上游单元的 `y`，非严格"上一个喂下一个"）；单元自身状态（PID 积分等）留在各自实例里。本轮单模态、单条静态 DAG，**不引入共享黑板**。

```python
# 僚机本体
def step(self, ctx: FormationAlgorithmContext) -> FormationAlgorithmOutput:
    parsed = self.rx.step(ctx.inbox)                       # 收发(收)(流程库)
    mode   = self.orch.step(parsed)                        # 任务编排(流程库, 僚机 Mode 来自广播, 恒"保持")
    plan   = self.planner.step((mode, parsed))             # 轨迹规划(流程库)
    target = self.possolve.step((plan, parsed.leader_nav)) # 位置解算(算法库) → Target
    dev    = self.devcalc.step((ctx.self_state, target))   # 误差解算(算法库)
    accel  = self.tracker.step((dev, ctx.self_state))      # 跟踪(算法库, 内部调控制算法) → AccelerationCommand
    return FormationAlgorithmOutput(control=accel, outbox=[], status=self._status())
```

长机本体在**一个对象**内同时跑飞行流与协调流，共享自己的状态（详见 `5-用例-领航跟随保持.md` 的长机 `step`）。

## 4. 生命周期

| 接口 | 语义 |
| --- | --- |
| `init` | 实例化并挂接单元，注入静态参数 |
| `step` | 一拍：消费注入的 `ctx`，编排单元，返回 `{control?, outbox, status}` |
| `read_state` | 汇出实体（及各子单元）状态，供落盘 / 快照（散落在子对象，需统一暴露） |
| `reset` | 回到初始：**整实体重新实例化**（最省事，与 `1-仿真控制HLD` 的 `reset() 重建内部模块` 一致），或逐单元 `reset` |
| `close` | 释放资源 |

## 5. TODO

- **编排丰富 / 动态管线**：本轮编排占位恒"保持"、`step()` 内固定顺序串联各单元；出现真实模态决策 / 异构僚机时，编排长出真实逻辑、固定串联让位给按模态选管线（届时再评估黑板，见下条）。
- **黑板 / 动态数据上下文**：与"编排抽离"连体，动态重连出现时再评估是否替换显式传参。
