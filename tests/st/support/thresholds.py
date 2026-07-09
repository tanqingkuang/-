"""ST 阈值集中表。注意：用户后续只需要调这个文件。"""

from __future__ import annotations

# 防撞距离：当前编队最小槽位间距约 40m 以上，20m 用于捕捉明显碰撞且不过度约束队形样式。
COLLISION_DISTANCE_M = 20.0

# 终点水平误差：ST 场景用短航线，末端允许 220m 过渡误差，避免控制器按固定时长结束时轻微欠/过飞误报。
TERMINAL_ERROR_M = 220.0

# 末端航向夹角：直线或末段任务结束时，速度方向应基本对准末航段；20 度为经验值待整定。
TERMINAL_HEADING_DEG = 30.0

# 队形稳态误差：后半段平均/末端槽位误差的容许值，首期按 220m 经验值锁定明显失控。
FORMATION_ERROR_M = 220.0

# 瞬移检测裕度系数：按配置速度上限和日志采样间隔放大，1.8 覆盖转弯与日志舍入误差。
TELEPORT_MARGIN_FACTOR = 1.8

# 单帧高度跳变裕度系数：按最大升降率放大，避免 0.1s 日志采样与四舍五入造成误报。
ALTITUDE_JUMP_MARGIN_FACTOR = 2.0

# 单帧速度跳变裕度系数：按加速度限幅放大，首期仅捕捉明显数值突变。
SPEED_JUMP_MARGIN_FACTOR = 2.0

# 物理限幅数值容差：日志已做小数舍入，保留 1e-2 量级余量。
LIMIT_EPS = 0.05

# 日志采样周期：当前控制器 snapshots.jsonl 固定 10Hz 落盘，不按 step_s 每积分步落盘。
LOG_SAMPLE_PERIOD_S = 0.1

# T2 完成时间容差：只允许完成时间相对基线恶化 10%，变好仅提示刷新。
METRIC_COMPLETION_TIME_TOLERANCE_RATIO = 0.10

# T2 最小机间距容差：只允许安全距离相对基线缩水 10%，变大仅提示刷新。
METRIC_DISTANCE_SHRINK_TOLERANCE_RATIO = 0.10

# T2 最小障碍裕度容差：只允许避障裕度相对基线缩水 10%，变大仅提示刷新。
METRIC_OBSTACLE_MARGIN_SHRINK_TOLERANCE_RATIO = 0.10

# T3 采样周期：紧凑轨迹每 1s 采一帧，降低基线体积并保留定位能力。
T3_SAMPLE_PERIOD_S = 1.0

# T3 位置舍入位数：厘米级足够识别轨迹漂移，同时屏蔽浮点尾差。
T3_POSITION_DECIMALS = 2

# T3 角度舍入位数：0.01 度可读且能定位航向变化。
T3_ANGLE_DECIMALS = 2

# T3 速度舍入位数：0.01m/s 对 ST 回归足够敏感。
T3_SPEED_DECIMALS = 2

# T3 字段清单：固定顺序输出，避免字典顺序或新增字段造成基线噪声。
T3_FIELDS = ("x_m", "y_m", "altitude_m", "psi_v_deg", "speed_mps")

# T3 数值比较容差：紧凑值已舍入，0.005 只屏蔽半单位尾差。
T3_COMPARE_EPS = 0.005

# WARN 白名单：当前首期要求无 WARN；后续若有可接受告警，在这里按消息片段加入。
ALLOWED_WARN_MESSAGE_PARTS: tuple[str, ...] = ()


