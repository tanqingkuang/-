# CLI HLD

## 1. 定位

CLI 是人机交互组的 Boundary，面向 headless 单进程仿真和脚本层批量仿真。CLI 负责解析命令行参数、加载配置、应用 CLI 覆盖，并调用仿真控制运行到结束。

## 2. 职责

- 解析 `--config`、`--seed`、`--output`、`--headless` 等命令行参数。
- 加载基础配置文件。
- 将命令行参数覆盖到配置对象。
- 调用 `仿真控制.start(config)`。
- 将进程退出码反馈给外层脚本。
- 支持 bat / shell / GNU parallel / xargs 等脚本层批量启动。

## 3. 边界

- 不感知“批量”概念，批量循环、并发、重试和资源限流由脚本层负责。
- 不启动控制界面或实时显示。
- 不直接调用算法、模型、通信或加扰。
- 不直接实现日志落盘，日志由仿真控制调度数据组完成。

## 4. 主要接口类别

- 参数解析：命令行参数、默认值、必填项校验
- 配置覆盖：seed、output、headless 等 CLI 覆盖
- 运行入口：调用仿真控制 `start(config)`
- 进程反馈：退出码、错误摘要

## 5. 关联代码

- `src/ui/cli/`
- `src/main.py`
- `src/runner/sim_control.py`

