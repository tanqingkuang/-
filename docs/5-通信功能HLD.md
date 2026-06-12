# 通信功能 HLD

## 1. 定位

通信功能属于仿真环境组，负责按拓扑和 QoS 配置路由消息，并维护 Inbox、Outbox 和在途队列。

## 2. 职责

- 接收仿真控制写入的算法 Outbox。
- 按 star、ring、mesh、稀疏图、时变图等拓扑路由消息。
- 按 init QoS 配置和 seed 实施链路时延、丢包、带宽等 stochastic 扰动。
- 接收加扰 push 的链路故障状态。
- 向仿真控制提供各算法实例的 Inbox。
- 只识别通用 envelope，不理解 payload 内的算法语义。

## 3. 边界

- 不读取全局配置文件。
- 不调用算法，不解释算法 payload。
- 不持有加扰引用，只暴露链路故障注入接口。
- 不负责模型状态积分或传感器噪声。

## 4. 主要接口类别

- 生命周期：`init`、`close`
- tick 节拍：`tick`
- 消息写入：`push_outbox`
- 消息读取：`read_inbox`
- 拓扑更新：`update_topology`
- stochastic 配置：`set_qos`
- 动态注入：`inject_link_fault`

## 5. 关联代码

- `src/environment/comm.py`
- `src/common/envelope.py`

