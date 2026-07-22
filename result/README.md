# 一键仿真

直接双击 `run_batch.bat`。脚本优先使用项目 `.venv`，通过 `src/main.py` 默认以 20 倍速从源码无界面运行；集结场景会自动触发集结。该过程不会启动 GUI、生成 exe 或安装依赖。

脚本默认运行 `configs/rally_demo_5_aircraft.json`，仿真数据写入
`result/simulation_data/logs/run-seed-<seed>-*`。

也可以在命令行传入其他仿真 JSON：

```bat
result\run_batch.bat configs\base.json
```

第二个参数可以覆盖默认倍率，例如以 5 倍速运行：

```bat
result\run_batch.bat configs\base.json 5
```

第三个参数指定不确定性 seed 列表。单独运行 seed 2：

```bat
result\run_batch.bat configs\base.json 50 2
```

`run_batch.bat` 默认并发启动 seed 0、1、2、3、4 五个最小化进程：

```bat
result\run_batch.bat
```

该 BAT 是异步启动器：负责展开 seed 列表并发起各仿真进程，全部进程成功发起后即完成启动职责，
不主动等待子进程结束，也不汇总子进程退出码。单个 seed 的运行结果以对应进程输出和
`run-seed-<seed>-*` 目录内的日志为准。

三个参数依次为配置、倍率和 seed 列表。seed 列表含空格时需要加双引号：

```bat
result\run_batch.bat configs\base.json 50 "0 1 2 3 4"
```

每次运行分别生成 `snapshots_seed_0.jsonl`、`snapshots_seed_1.jsonl` 等文件；
运行目录也包含对应 seed，批量并发时不会混淆结果。

当前不确定性算例：

- `seed=0`：标称状态。
- `seed=1`：全链路丢包率 2.3%。
- `seed=2`：全局轻度紊流风（水平标准差 0.8 m/s、垂向标准差 0.3 m/s、相关时间 2 s）。
- `seed=3`：全链路发送帧频限制为 10 Hz（标称算法发送节拍为 20 Hz）。
- `seed=4`：全链路时延设置为 50 ms。

控制效果分析与仿真分开执行。双击 `analyze_accuracy.bat` 后选择某次仿真目录中的
`snapshots_seed_<seed>.jsonl`；也可以直接传入快照文件：

```bat
result\analyze_accuracy.bat
result\analyze_accuracy.bat result\simulation_data\logs\run-seed-2-1784530025\snapshots_seed_2.jsonl
```

分析报告写入 `result/analysis/run-*/`：

- `control_effect_metrics.csv`：使用 `src.data.control_effect_analysis` 统一生成全机和逐机指标。

报告覆盖快照的完整时间范围，包含位置/速度跟踪误差、刚性槽位误差、航线横偏、过载、控制指令和算法耗时等已有通道；各通道统一输出均值、方差、标准差、均方根、最大绝对值、绝对值 95 分位、总变差和积分等指标。
