"""二维平面避障（A*）后端。注意：纯函数实现，与仿真主循环解耦，便于单测。

模块划分（见 docs/避障-A星-开发计划.md §5）：
- obstacle.py    障碍数据结构 ObstacleS + 唯一形状基元 inside()（圆 / 矩形）
- astar.py       栅格化 + A* 内核 plan_path()，格子判定复用 inside()
- path_to_route.py  （步骤3）去冗余 + 圆弧 → RouteS
- feasibility.py    （步骤4）可飞性校验（腿长 + 圆弧采样逐点 inside()）
"""
