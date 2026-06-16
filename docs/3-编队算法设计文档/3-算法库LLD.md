# 算法库 LLD

> 算法库是一组**不碰 `Mode` 的纯计算类**（实例态在 `self`、无全局态），跨方法复用；由流程在外部按 Mode 选 / 切（分界见 `0-HLD.md` 原则 9）。
> 单元小协议见 `1-LLD综述.md` §2.1：`init(params) / step(u)->y / reset / read_state`。
> **每新增一个算法，在此册增改对应单元。**

## 1. 单元清单（4 个策略族 + 通用数学）

| 族（抽象基类） | 单元 | `u` → `y` | 本轮 |
| --- | --- | --- | --- |
| `PositionSolve` | 位置解算 | `(Plan, leader_nav?)` → `Target` | ✅ |
| `DeviationCalc` | 误差解算 | `(self_state, Target)` → `Deviation`（侧偏 / 待飞距 / 航迹角偏差 / 曲率） | ✅ |
| `Tracking` | 轨迹 / 编队跟踪 | `(Deviation, self_state)` → `AccelerationCommand`；**按通道组合控制算法** | ✅ |
| `ControlLaw` | 控制算法 | 控制误差 → 控制量（被 `Tracking` 组合调用） | ✅ |
| —（工具） | 通用数学 | 坐标变换、限幅、滤波；（未来）一致性律 | 按需 |

- **位置解算实现**：航线插值（长机，吃 `Plan` 的航段）/ 槽位几何（僚机，吃队形 + 槽位号 + `leader_nav`）/（未来）Dubins。
- **跟踪实现**：不同通道组合（如垂向 TECS、横侧 PID）；跟踪本身不挑模态，挑哪套组合由流程在外部切。
- **控制算法实现**：PID / L1 / ADRC 等原子律。

> 输出口径对齐 `../2-模型迭代HLD.md`：跟踪链最终产出 `AccelerationCommand{ax/ay/az_cmd_mps2}`（ENU）。

## 2. 单元规范（统一）

- **类、无全局态**：所有可变状态放 `self`；同一实体可挂同类多个实例（如三轴各一个控制算法），互不干扰。
- **不碰 `Mode`**：不读任务状态机；"用哪个策略"由流程在外部选 / 切，单元只管纯计算。
- **C 友好**：`PID(gains)` ≡ `pid_init(pid_t*, gains)`，`self.pid.step(e)` ≡ `pid_step(pid_t*, e)`。

## 3. TODO

各单元的**数学实现（方程 / 参数 / 限幅 / 默认增益）逐个补**：

- [ ] 位置解算：航线插值；槽位几何（wedge / line / echelon 的槽位号 → 相对偏置）
- [ ] 误差解算：侧偏 / 待飞距 / 航迹角偏差的几何定义
- [ ] 跟踪：通道分配与组合（垂向 / 横侧选哪套控制算法）
- [ ] 控制算法：PID 增益 / 积分限幅 / 抗饱和；（未来）L1 / ADRC 选型