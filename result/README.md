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

编队精度分析与仿真分开执行。双击 `analyze_accuracy.bat` 后选择某次仿真目录中的 `snapshots.jsonl`；也可以直接传入快照文件：

```bat
result\analyze_accuracy.bat
result\analyze_accuracy.bat result\simulation_data\logs\run-1784530025\snapshots.jsonl
```

分析报告写入 `result/analysis/run-*/`：

- `formation_accuracy_detail.csv`：每架僚机一行的编队位置与位置跟踪指标。

报告先要求全队任务状态同时为 `HOLD`，再要求全队最差三维编队位置误差连续满足配置门限。只有稳定判定完成后的数据才进入统计；目录中只生成逐僚机指标 CSV，不再生成汇总 CSV 和 JSON。
