# 一键仿真

直接双击 `run_batch.bat`。脚本优先使用项目 `.venv`，通过 `src/main.py` 默认以 10 倍速从源码无界面运行；集结场景会自动触发集结。该过程不会启动 GUI、生成 exe 或安装依赖。

脚本默认运行 `configs/rally_demo_5_aircraft.json`，仿真数据写入 `result/simulation_data/logs/run-*`。

也可以在命令行传入其他仿真 JSON：

```bat
result\run_batch.bat configs\base.json
```

当前脚本完成单份 JSON 的源码无界面一键仿真；后续批量不确定性任务将在该入口上扩展。

指定其他配置：

```bat
result\run_batch.bat configs\base.json
```

第二个参数可以覆盖默认倍率，例如以 5 倍速运行：

```bat
result\run_batch.bat configs\base.json 5
```
