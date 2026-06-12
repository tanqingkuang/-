# 模型迭代 HLD

## 1. 定位

模型迭代属于仿真环境组，负责统一推进所有飞机的动力学积分，并在状态读出和控制输入应用阶段体现模型侧不确定性。

## 2. 职责

- 按仿真控制给定的控制量推进飞机状态。
- 管理飞机动力学状态，例如位置、速度、姿态、角速度等。
- 按 init 配置和 seed 实施传感器噪声、定位漂移、控制滞后等 stochastic 扰动。
- 接收加扰 push 的风扰、单机故障等动态扰动。
- 向仿真控制提供算法所需的状态读出接口。

## 3. 边界

- 不读取全局配置文件，配置由仿真控制 / 加扰分段下发。
- 不持有算法引用，不调用算法。
- 不持有加扰引用，只暴露被扰输入接口。
- 不负责通信链路模拟。

## 4. 主要接口类别

- 生命周期：`init`、`close`
- tick 节拍：`tick`
- 状态读写：`read_states`、`apply_controls`
- stochastic 配置：`set_sensor_noise`、`set_position_drift`、`set_control_lag`
- 动态注入：`inject_wind`、`inject_fault`

## 5. 关联代码

- `src/environment/model.py`

