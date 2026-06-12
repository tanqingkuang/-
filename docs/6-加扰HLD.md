# 加扰 HLD

## 1. 定位

加扰是高层分割阶段新增的横切 Control，统一管理不确定性索引、stochastic 扰动配置分发和动态扰动推进。

## 2. 职责

- 接收扰动配置和不确定性索引。
- 根据 seed / 不确定性索引展开可复现的不确定性组合。
- init 时将 stochastic 扰动配置分发给模型迭代和通信功能。
- 运行期 `tick` 推进风扰、单机故障、链路故障等动态扰动。
- 通过 `inject_*` 接口将动态扰动 push 给模型迭代或通信功能。
- 接收仿真控制转发的 UI 动态扰动注入命令。

## 3. 边界

- 不调度算法、模型或通信的主 tick。
- 不读取算法状态语义。
- 不写关键数据，但其实际不确定性索引和扰动状态可由仿真控制记录。
- model / comm 不反向调用加扰。

## 4. 主要接口类别

- 生命周期：`init`、`close`
- tick 节拍：`tick`
- 动态命令：`inject_wind`、`inject_fault`、`inject_link_fault`
- stochastic 分发：模型侧扰动配置、通信侧 QoS 扰动配置

## 5. 关联代码

- `src/environment/disturb.py`

