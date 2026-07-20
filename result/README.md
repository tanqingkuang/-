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

控制效果分析与仿真分开执行。双击 `analyze_accuracy.bat` 后选择某次仿真目录中的 `snapshots.jsonl`；也可以直接传入快照文件：

```bat
result\analyze_accuracy.bat
result\analyze_accuracy.bat result\simulation_data\logs\run-1784530025\snapshots.jsonl
```

分析报告写入 `result/analysis/run-*/`：

- `control_effect_metrics.csv`：使用 `src.data.control_effect_analysis` 统一生成全机和逐机指标。

报告覆盖快照的完整时间范围，包含位置/速度跟踪误差、刚性槽位误差、航线横偏、过载、控制指令和算法耗时等已有通道；各通道统一输出均值、方差、标准差、均方根、最大绝对值、绝对值 95 分位、总变差和积分等指标。
