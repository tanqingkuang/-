# CLI HLD

## 1. 定位

CLI 是人机交互组的 Boundary，面向 headless 单进程仿真和脚本层批量仿真。headless 运行以 `SimulationController.run_until_complete(config, seed=seed)` 为同步执行入口；命令行包装负责把外部参数转换为配置对象和运行参数，再调用该入口运行到结束。

## 2. 职责

- 解析 `--config`、`--seed`、`--output`、`--headless` 等命令行参数。
- 加载基础配置文件。
- 将命令行参数覆盖到配置对象。
- 调用 `SimulationController.run_until_complete(config, seed=seed)`；seed 是默认值为 0 的非负整数，配置文件中的同名历史字段不参与选择。
- 将进程退出码反馈给外层脚本。
- 支持 bat / shell / GNU parallel / xargs 等脚本层批量启动。

## 3. 边界

- 不感知“批量”概念。仓库提供的 `result/run_batch.bat` 仅负责展开 seed 列表并异步发起并发进程，不等待、重试或汇总子进程退出码；需要这些能力时由上层任务编排器负责。
- 不启动控制界面或实时显示。
- 不直接调用算法、模型、通信或加扰。
- 不直接实现日志落盘，日志由仿真控制调度数据组完成。

## 4. 主要接口类别

- 参数解析：命令行参数、默认值、必填项校验
- 配置覆盖：seed、output、headless 等 CLI 覆盖
- 运行入口：调用仿真控制 `run_until_complete(config, seed=seed)`
- 进程反馈：退出码、错误摘要

## 5. 关联代码

- `src/ui/cli/`
- `src/main.py`
- `src/runner/sim_control.py`
