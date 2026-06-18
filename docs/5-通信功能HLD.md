# 通信功能 HLD

## 1. 定位

通信功能属于仿真环境组，模拟编队内无人机之间的无线链路。负责消息的路由、延迟仿真和丢包仿真，不关心消息内容语义。详细设计见 [5-1-通信功能LLD.md](5-1-通信功能LLD.md)。

## 2. 职责

- 接收仿真控制写入的算法 Outbox，按消息的 `target` 字段投递，不参与路径规划。
- 按链路配置的延迟和丢包率仿真物理信道；未配置的链路视为不通。
- 所有消息经在途队列缓冲，最早下一 tick 可读，保证帧因果关系。
- 支持运行期链路 QoS 更新（latency_ms / loss_rate）；节点和链路数量在 init 时固定，运行期不可增删；reset 时恢复 init 基线。
- 接受加扰模块注入链路断链/恢复（支持有时限的自动恢复）。
- 向仿真控制提供各节点的 Inbox 读取接口和链路状态查询接口。

## 3. 边界

- 不读取全局配置文件，拓扑和 QoS 由仿真控制在 `init` 时下发。
- 不解析 `MessageEnvelope.payload`。
- 不持有算法或加扰模块引用，只暴露被动接受接口。
- 节点数量在 `init` 时固定，运行期不变。
- 当前不仿真带宽限制。
- 不参与算法层的路径规划；多跳路由由算法层决定，通信层只做逐跳投递。

## 4. 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 链路方向 | 支持 `duplex` / `simplex` | 同时覆盖双向无线链路和单向受限链路场景 |
| 未配置链路 | 视为不通，静默丢弃 | 保证稀疏拓扑下链路状态可见、可注入故障 |
| 零延迟消息路径 | 统一走在途队列 | 避免同 tick 内发送即可读，保证帧因果关系 |
| reset 语义 | 恢复 init 基线 | 保证多轮仿真复现性；运行期修改不进基线 |
| 链路 ID 归一化 | `duplex` 接受正反两向；`simplex` 只接受精确方向 | 避免单向链路被误当成双向链路 |

## 5. 主要接口

| 类别 | 接口 |
|------|------|
| 生命周期 | `init(config, seed)` · `reset()` · `close()` |
| 拓扑管理 | `update_topology(config)` |
| 主循环 | `send(messages)` · `tick(dt_s)` · `read_inbox(node_id)` |
| 状态查询 | `read_link_states()` |
| 故障注入 | `inject_link_fault(link_id, status, duration_s)` · `inject_link_qos(link_id, latency_ms, loss_rate)` |

## 6. 关联代码

- `src/environment/comm.py`
- `src/common/envelope.py`
