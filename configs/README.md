# 配置说明

本目录记录每个 JSON 配置文件对应的仿真内容。客户可编辑的基础航线和避障障碍文件使用经纬高；控制器和算法内部仍统一采用 ENU：`x_m` 为东向，`y_m` 为北向，`altitude_m` 为高度；外部队形文件中的槽位使用 `x_forward_y_up_z_right`。

## `base.json`

- 默认三机演示场景。
- 航线速度为 `20 m/s`，包含东向平飞后北向平飞的基础折线航线。
- 基础航线从 `element/line.json` 引用，主配置只保留 `route_file` 相对路径。
- 避障障碍库从 `element/obstacles.json` 引用，主配置只保留 `avoidance.obstacles_file` 相对路径。
- 编队从 `element/formations/triangle_3_aircraft_a01_a03.json` 引用，主配置只保留 `formation.formation_files` 相对路径。
- 编队为 1 架长机加 2 架僚机的三角队形，用于 GUI、控制器和基础回归验证。

## `element/line.json`

- 默认基础航线文件，结构与原 `base.json.route` 完全一致。
- 对外使用经纬高：`latitude_deg`、`longitude_deg`、`altitude_m`。加载时以第一个航点为 ENU origin，转换后的高度仍取 `altitude_m` 原值。
- `route_file` 的解析和生成由 `src/data/linefile/` 下的策略工厂负责，设计说明见 `src/data/linefile/航线文件设计.md`。

## `rally_demo.json`

- 三机分散后执行集结并进入任务航线的演示场景。
- 任务航线从 `element/rally_demo_rally_route.json` 引用，主配置只保留 `route_file` 相对路径。
- 集结长机航线从 `element/rally_demo_rally_route.json` 引用，主配置只保留 `rally_route_file` 相对路径。
- 两个外部航线文件加载后分别展开为控制器和算法消费的 `route` 与 `rally_route`。
- 集结目标队形复用 `element/formations/triangle_3_aircraft_a01_a03.json`。

## `element/obstacles.json`

- 默认避障障碍库文件，结构与原 `base.json.avoidance.obstacles` 完全一致。
- 对外使用经纬度表达障碍位置：圆形障碍的半径仍为米，矩形障碍使用四点经纬度，可表达旋转矩形；加载时由上层注入基础航线 origin 并转换为 ENU。
- `obstacles_file` 的解析和生成由 `src/data/obstaclefile/` 下的策略工厂负责，设计说明见 `src/data/obstaclefile/障碍文件设计.md`。

## `single_avoidance_80km.json`

- 单机避障演示场景，不配置编队通信链路。
- 基础航线从 `element/single_avoidance_80km_route.json` 引用，包含 10 个经纬高航点、9 条直线航段，航段长度约 `11 km` 到 `24 km`，整体视野约 `80 km`。
- 原航线内部拐点默认转弯半径为 `700 m`；飞机初始位于第一航点，速度 `45 m/s`，航向与第一航段一致。
- 避障障碍库从 `element/single_avoidance_80km_obstacles.json` 引用，包含 2 个圆形、2 个轴对齐矩形和 2 个斜放矩形。
- 避障规划参数使用 `turn_radius_m=600 m`、`leg_length_margin_m=0 m`、`grid.resolution_m=50 m`，其余参数按避障设计文档建议给出初始默认值。

## `diamond.json`

- 钻石项目 XML 航线格式演示场景，整体仿真参数复用 `single_avoidance_80km.json`。
- 基础航线从 `element/航线1 钻石默认航线 2026年7月6日10时8分18秒.XML` 引用，航点坐标复用 `single_avoidance_80km_route.json`，由钻石 XML 策略在加载时补 `45 m/s` 航线速度。
- 避障障碍库复用 `element/single_avoidance_80km_obstacles.json`，避免重复维护同一批障碍。
- `avoidance.allow_arc=false`，使默认生成的避障航线保持折线输出，便于直接导出为钻石 XML。

## `quadrilateral_10_aircraft_a05_leader.json`

- 十机四边形航线场景，航段速度为 `20 m/s`。
- 航线按顺序构成平飞、爬升、平飞、下滑四个动作：第二段从 `1000 m` 爬升到 `1100 m`，第四段从 `1100 m` 下滑回 `1000 m`。
- 爬升段和下滑段水平距离均为 `2000 m`，按 `20 m/s` 约需 `100 s`，对应垂向速度约 `1 m/s`。
- 飞机模型爬升和下滑限幅均设为 `4 m/s`，高于航线目标垂向速度，便于观察限幅裕度。
- 十架飞机采用 `1、2、3、4` 四行三角编队。前后行距为 `40 m`，按等边三角形关系计算同行横向间距：`2 * 40 / sqrt(3) = 46.19 m`。
- 队形槽位从 `element/formations/triangle_10_aircraft_a01_a10.json` 引用。
- 初始编队中心位于第一航段反向延长线上，节点带有小幅航迹角、高度、侧偏和速度偏差，用于观察收敛过程。
- A05 作为 `leader` 和队形参考原点，A01 改为前方槽位，其他槽位整体按 A05 原槽位平移重算。
- 通信链路采用 A05 星形拓扑，保证僚机能接收 A05 长机广播。

## `change.json`

- 五机队形热切换演示场景，航线为直线任务航线。
- 队形列表按 `formation.formation_files` 顺序决定索引：`0` 为 `element/formations/wedge_5_aircraft_a01_a05.json`，`1` 为 `element/formations/line_abreast_5_aircraft_a01_a05.json`，`2` 为 `element/formations/double_column_5_aircraft_a01_a05.json`。
- 运行时切换只广播队形索引，算法层按展开后的 `formPat/formPos` 查槽位。

## `element/formations/*.json`

- 每个文件只描述一个队形，根对象包含 `name` 和全量 `slots`。
- `slots` 必须覆盖总配置 `nodes` 中的全部节点，槽位字段使用 `node_id/x_m/y_m/z_m`。
- 主配置必须通过 `formation.formation_files` 引用队形文件；不再支持在总配置里内联 `formation.pattern`、`formation.slots` 或 `formation.formations`。
