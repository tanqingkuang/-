# 领航跟随集结 LLT 设计

测试文件：

- `tests/llt/test_formation_rally.py`
- `tests/llt/test_sim_control_rally.py`

被测模块：

- `src/algorithm/context/leaf_types.py`
- `src/algorithm/context/context.py`
- `src/algorithm/entity/types.py`
- `src/algorithm/units/process/formation_task/rally.py`
- `src/algorithm/units/process/outbound/follower_broadcast.py`
- `src/algorithm/units/process/outbound/rally_leader_broadcast.py`
- `src/algorithm/units/process/inbound/follower_status.py`
- `src/algorithm/units/process/inbound/rally_leader_follower.py`
- `src/algorithm/units/algo/pos_calc/rally_join_pos.py`（原 `rally_approach.py`，已整体替换并删除）
- `src/algorithm/units/algo/pos_calc/slot_geometry.py`（CATCHUP/LOOSE/HOLD 共用；原 `catchup_align.py` 已删除）
- `src/algorithm/entity/leader_follower/leader.py`
- `src/algorithm/entity/leader_follower/follower.py`
- `src/runner/sim_control_modules.py`、`src/runner/sim_control_routes.py`、`src/runner/sim_controller.py`（原 `sim_control.py`，已拆分为多个模块）

设计依据：`docs/3-编队算法设计文档/6-用例-领航跟随集结LLD.md`

---

## 测试辅助

> 下列辅助函数摘自 `tests/llt/test_formation_rally.py` 当前实现（原文档版本节点用 `leader`/`follower_1`
> 等占位名、且 `_rally_task`/`FollowerStateS` 还带着已删除的 `arrive_hold_s`/`arriveHold_s`，与实际测试
> 文件的三机 `R01/R02/R03` 命名和当前 `RallyTaskInitS` 字段严重脱节，此处按实际代码重写）：

```python
def _pos(east: float = 0.0, north: float = 0.0, h: float = 0.0) -> PosInEarthS:
    return PosInEarthS(east=east, north=north, h=h)


def _motion(
    east: float = 0.0,
    north: float = 0.0,
    h: float = 0.0,
    v_east: float = 0.0,
    v_north: float = 0.0,
    v_up: float = 0.0,
    vd: float | None = None,   # None 时取水平速度模长 hypot(v_east, v_north)
    v_psi: float = 0.0,
    d_v_psi: float = 0.0,
) -> MotionProfS:
    ground_speed = math.hypot(v_east, v_north) if vd is None else vd
    return MotionProfS(
        pos=PosInEarthS(east=east, north=north, h=h),
        v=VdInEarthS(vEast=v_east, vNorth=v_north, vUp=v_up, vd=ground_speed, vPsi=v_psi, dVPsi=d_v_psi),
    )


def _follower_state(
    node_id: str,
    *,
    pos_err_m: float = 0.0,
    last_update_s: float = 0.0,
    rally_state: str = "EXITED",
    planned_path_length_m: float = -1.0,
) -> FollowerStateS:
    return FollowerStateS(
        id=node_id, posErr_m=pos_err_m, lastUpdate_s=last_update_s,
        rally_state=rally_state, plannedPathLength_m=planned_path_length_m,
    )


def _follower_status_msg(
    source: str = "R02",
    *,
    pos_err_m: float = 0.0,
    heading_err_rad: float = 0.0,
    rally_state: str = "EXITED",
    planned_path_length_m: float = -1.0,
) -> MessageEnvelope:
    return MessageEnvelope(
        topic=FOLLOWER_STATUS_TOPIC, source=source, target="R01", timestamp=0.0,
        payload={
            "pos_err_m": pos_err_m, "heading_err_rad": heading_err_rad,
            "rally_state": rally_state, "planned_path_length_m": planned_path_length_m,
        },
    )


def _leader_msg(
    *,
    stage: FormStageE = FormStageE.RALLY,
    pattern: int = 0,
    step: int = 0,
    leader_state: MotionProfS | None = None,
    t_ref: float = 0.0,
    t_ref_valid: bool = True,
) -> MessageEnvelope:
    state = leader_state or _motion(east=100.0, north=200.0, h=500.0, v_east=20.0)
    return MessageEnvelope(
        topic="formation.leader", source="R01", target=["R02"], timestamp=0.0,
        payload={
            "leader_state": _motion_payload(state),
            "cmd": {"stage": int(stage), "pattern": int(pattern), "step": step},
            "t_ref": t_ref,
            "t_ref_valid": t_ref_valid,
        },
    )


def _rally_task(
    expected: tuple[str, ...] = ("R02", "R03"),
    *,
    dt_s: float = 0.1,
    stable_hold_s: float = 0.2,
    catchup_radius_m: float = 200.0,
    catchup_stable_s: float = 0.0,
) -> Rally:
    task = Rally()
    task_cfg = RallyTaskInitS(
            looseScale=3.0, convergenceRadius_m=5.0, stableHold_s=stable_hold_s,
            tightRadius_m=2.0, expectedFollowerIds=list(expected),
            staleTimeout_s=0.5, targetPattern=0, dt_s=dt_s,
            catchup_radius_m=catchup_radius_m, catchup_stable_s=catchup_stable_s,
    )
    task.init(
        EntityManagerInitS(
            EntityInitS(selfInit=FormSelfInitS("R01"), rally_cfg=task_cfg),
            LEADER_PROFILE,
        )
    )
    return task


def _task_step(
    task: Rally,
    ctx: FormContextS,
    *,
    remote: FormStageE,
    states: list[FollowerStateS] | None = None,
    now_s: float = 0.0,
    leader_exited: bool = True,
) -> RallyTaskOutputS:
    """推进 Rally 任务一拍并返回输出端口。"""
    output = RallyTaskOutputS(cmd=ctx.cmd, rallyPlan=ctx.rallyPlan)
    task.step(
        RallyTaskInputS(
            remote=RemoteCmdS(remote), cmd=ctx.cmd, followerStates=states or [],
            clock=ctx.clock, posCalcStatus=ctx.posCalcStatus,
        ),
        output,
    )
    return output


def _comm_init() -> FormCommInitS:
    """构造三机集结通信与三角队形槽位。"""
    return FormCommInitS(
        netWork=[NetWorkS("R01", "R02", CommDirE.DUPLEX), NetWorkS("R01", "R03", CommDirE.DUPLEX)],
        formPat=[0],
        formPos=[[
            FormPosS("R01", 0.0, 0.0, 0.0),
            FormPosS("R02", -10.0, 0.0, -5.0),
            FormPosS("R03", -10.0, 0.0, 5.0),
        ]],
    )


def _route(start: tuple, end: tuple, speed: float = 20.0) -> list[WayPointInputS]:
    """构造两点航线（WayPointInputS 列表）。"""
    return [
        WayPointInputS(pos=PosInEarthS(*start), vdCmd=speed),
        WayPointInputS(pos=PosInEarthS(*end), vdCmd=speed),
    ]


def _rally_cfg(
    *,
    expected: tuple[str, ...] = ("R02", "R03"),
    dt_s: float = 0.1,
    stable_hold_s: float = 0.1,
    catchup_stable_s: float = 0.0,
) -> RallyTaskInitS:
    """构造实体测试用集结配置。"""
    return RallyTaskInitS(
        looseScale=3.0, convergenceRadius_m=5.0, stableHold_s=stable_hold_s,
        tightRadius_m=2.0, expectedFollowerIds=list(expected),
        staleTimeout_s=1.0, targetPattern=0, dt_s=dt_s, catchup_stable_s=catchup_stable_s,
    )
```

---

## 1. TestRallyLeafTypesAndContext - 叶类型与 Context

| 测试名 | 断言 |
| ------ | ---- |
| `test_leaf_types_only_keep_runtime_consumed_fields` | `FollowerStateS`、`PosCalcStatusS` 与航段类型只保留运行期实际消费字段 |
| `test_rally_leaf_type_defaults_and_copy_helpers` | `copy_follower_state` 完整复制精简后的六个字段 |
| `test_context_contains_rally_fields` | `FormContextS` 含公共计划与 `followerStates`，且列表默认独立 |
| `test_reset_context_clears_rally_state` | `reset_context` 后公共计划和 `followerStates` 恢复默认值 |

---

## 2. TestEntityBoundaryTypes - 实体边界扩展

| 测试名 | 断言 |
| ------ | ---- |
| `test_entity_input_contains_now_s_default_zero` | `EntityInputS().now_s == 0.0` |
| `test_entity_boundary_defaults_include_rally_fields` | `EntityInitS.route` 默认空列表且不再含 `rally_route`；M_i 由 init 按统一 `route` 前两点和队形槽位自动推导，`rally_cfg/rally_approach_speed_mps/rally_leader_id` 默认值符合 LLD（`rally_leader_id=""` 为默认值） |
| `test_entity_boundary_defaults_include_rally_fields` | `EntityOutputS().rallyCompleted is False`，且不再暴露已删除的 `formationAnalysis` |
| `test_direct_hold_skips_rally_products_and_loiter_speed_validation` | 通用长机/僚机以 `rally_enabled=False` 直接保持时，不创建 RallyJoinPos，也不校验仅集结使用的盘旋速度范围 |

---

## 3. TestRallyTaskValidation - Rally 初始化校验

| 测试名 | 断言 |
| ------ | ---- |
| `test_task_config_excludes_entity_assembly_fields` | `RallyTaskInitS` 不再包含身份、启用状态和速度权限等实体装配字段 |
| `test_init_rejects_loose_scale_below_one` | `looseScale < 1.0` 抛 `ValueError` |
| `test_init_rejects_zero_stale_timeout` | `staleTimeout_s <= 0` 抛 `ValueError` |
| `test_init_rejects_zero_dt` | `dt_s <= 0` 抛 `ValueError` |
| `test_reset_restores_none_state` | `reset()` 清零计时器，下一次 `remote=NONE` step 输出 `NONE/step=0/pattern=NONE` |

---

## 4. TestRallyTaskRemote - Rally 遥控语义

| 测试名 | 断言 |
| ------ | ---- |
| `test_remote_none_outputs_none` | `remote=NONE` 输出 `cmd.stage=NONE`、`pattern=NONE` |
| `test_remote_none_resets_running_timers` | RALLY 中已累加计时器后切 `NONE`，再进 RALLY 不继承旧计时 |
| `test_remote_hold_outputs_hold` | `remote=HOLD` 输出 `HOLD` 和 `targetPattern` |
| `test_remote_hold_does_not_mark_rally_completed` | 外部强制 HOLD 不设置正常完成事件 |
| `test_remote_rally_from_none_starts_approach` | `NONE + remote=RALLY` 重置计时器并输出 `RALLY/step=0/targetPattern` |
| `test_remote_rally_after_completed_hold_does_not_restart` | 已正常完成到 `HOLD` 后继续 `remote=RALLY`，保持 `HOLD`，不回到 `APPROACH` |
| `test_none_then_rally_allows_restart` | 完成后先发 `NONE` 再发 `RALLY`，允许重新从 `APPROACH` 开始 |

---

## 5. TestRallyTask - Rally 状态机

| 测试名 | 断言 |
| ------ | ---- |
| `test_empty_expected_followers_advances_by_timers` | `expectedFollowerIds=[]` 时，APPROACH/LOOSE 可按计时器立即推进 |
| `test_missing_follower_states_freezes_approach` | 期望列表非空但 `followerStates=[]`，保持 `APPROACH`，`_arrive_timer` 不累加 |
| `test_approach_requires_all_expected_exited_and_fresh` | 全部参与者均为 `EXITED` 时才从 JOINING 推进；非终态回报还必须未超时 |
| `test_first_rally_frame_writes_target_pattern` | 第一拍 `RALLY/APPROACH` 即写 `cmd.pattern=targetPattern` |
| `test_loose_uses_position_error_threshold` | LOOSE 阶段按 `posErr_m < convergenceRadius_m` 判定稳定 |
| `test_loose_timer_resets_when_any_expected_follower_outside_radius` | 任一期望僚机误差超阈值，`_stable_timer` 清零 |
| `test_loose_stability_transitions_directly_to_hold` | 全部误差达标并持续 `stableHold_s` 后直接输出 `HOLD`，并只在转换拍输出 `rallyCompleted=True` |

---

## 6. TestFollowerBroadcast - 僚机状态广播

| 测试名 | 断言 |
| ------ | ---- |
| `test_targets_leader_from_cfg` | `cfg.leaderId` 非空时，`MessageEnvelope.target` 等于 `cfg.leaderId` |
| `test_broadcast_topic_is_follower_status` | 输出 topic 固定为 `formation.follower_status` |
| `test_follower_broadcast_targets_leader_and_reports_error` | payload 只含 `pos_err_m/heading_err_rad/rally_state/planned_path_length_m` |
| `test_pos_error_is_distance_to_self_cmd` | `pos_err_m = norm(selfState.pos - selfCmd.pos)` |
| `test_empty_leader_id_raises_value_error` | `cfg.leaderId == ""` 时 `init()` 抛 `ValueError`（不依赖 netWork 推断） |
| `test_missing_ports_raise_value_error` | `selfState/selfCmd/outbox` 未绑定时抛 `ValueError` |

---

## 7. TestFollowerStatusInbound - 长机解析僚机回报

| 测试名 | 断言 |
| ------ | ---- |
| `test_parses_two_follower_messages` | 两条 `formation.follower_status` 写入两个 `FollowerStateS`，字段完整 |
| `test_updates_existing_entry_in_place` | 已有同 ID 条目时原地更新，不追加重复项 |
| `test_appends_new_follower_entry` | 新 source 追加到 `followerStates` |
| `test_follower_status_parses_updates_filters_and_uses_source_id` | 成功解析即创建或原地更新状态条目并写入 `lastUpdate_s=now_s`；时效性仅由该时间戳判断 |
| `test_empty_inbox_keeps_last_update` | 断链帧不更新 `lastUpdate_s`，不清空旧状态 |
| `test_filters_non_follower_status_topics` | `formation.leader`、`node.status` 等非目标 topic 被忽略 |
| `test_ignores_non_dict_or_incomplete_payload` | payload 非 dict 或关键字段缺失时跳过，不写半截状态 |
| `test_payload_id_mismatch_uses_envelope_source` | payload 中 `id` 与 `envelope.source` 不一致时，以 `envelope.source` 作为 `FollowerStateS.id`，不信任 payload id |

---

## 8. TestRallyLeaderBroadcastAndInbound - 统一长机广播与僚机解析

| 测试名 | 断言 |
| ------ | ---- |
| `test_rally_leader_broadcast_keeps_existing_payload_contract` | payload 保留 `leader_state` 与 `cmd`，topic 仍为 `formation.leader` |
| `test_rally_leader_broadcast_adds_plan` | payload 包含公共到达时刻、有效标记和圈数计划 |
| `test_rally_leader_follower_parses_command_and_plan` | 入站单元同时写入 `leaderState/leaderCmd/cmd/rallyPlan`，缺少 `cmd["leader"]` 时 `leaderCmd` 回退为 `leaderState` |
| `test_non_leader_topic_is_skipped` | 非 `formation.leader` topic 不改写长机状态、命令和公共计划 |
| `test_latest_leader_message_wins` | 同帧多条长机广播，后到消息覆盖先到消息 |

---

## 9. TestRallyJoinPos - 切入盘旋圆与圆弧汇合

> 本节原名 TestRallyApproach，描述旧版"FLYING 直飞 M_i"的测试点；本次"切线进圆"重构后 FLYING 改为直飞
> 切入点 T，旧测试点已随之失效，替换为下列覆盖切入点/盘旋圆/切出航向的用例。均位于
> `tests/llt/test_formation_rally.py::RallyPosCalcTests`，直接单测 `RallyJoinPos`（不经实体/仿真控制器）。

**`loiter_speed_bounds()` 直接单测**（velCmdLimit → 盘旋速度上下限的推导与序校验，位于
`tests/llt/test_formation_rally.py::RallyLoiterSpeedBoundsTests`；回归覆盖"只显式配置一侧、另一侧退回
默认值导致反序"这一配置期漏判）：

| 测试名 | 断言 |
| ------ | ---- |
| `test_only_forward_max_configured_below_default_min_rejected` | 只配 `forwardMax=10`（< 默认 `loiter_min=14`）时，`(14, 10)` 是非法区间，`loiter_speed_bounds()` 显式拒绝 |
| `test_only_forward_min_configured_above_default_max_rejected` | 只配 `forwardMin=30`（> 默认 `loiter_max=25`）时，`(30, 25)` 是非法区间，`loiter_speed_bounds()` 显式拒绝 |
| `test_both_unconfigured_uses_valid_defaults` | 两侧都不配置时退回默认 `(14.0, 25.0)`，自洽，不报错 |
| `test_both_explicitly_configured_and_consistent_passes_through` | 两侧都显式配置且自洽（如 18/22）时原样透传，不受默认值影响 |

**`rally_loose_target()` 直接单测**（M_i 的 ENU 水平集结平面旋转/右侧轴符号/缩放/高度计算，位于
`tests/llt/test_formation_rally.py::RallyLooseTargetTests`；此前只被实体级测试间接覆盖，未单独验证过
旋转矩阵方向、右手轴符号、`looseScale` 是否只缩放水平分量）。这里的 M_i 是基于航线首段水平航向的
静态盘旋圆几何，等价于任务航向对齐、倾角为零的合成平飞 FUR，不是随当前航迹旋转的实时三维 FUR
槽位；进入 CATCHUP/LOOSE/HOLD 后由 `SlotGeometry` 接管三维变换：

| 测试名 | 断言 |
| ------ | ---- |
| `test_pure_forward_offset_at_zero_heading` | `heading=0`（正东）时，纯前向偏置（`slot.x`）直接映射为东向偏置，北向不变 |
| `test_right_axis_sign_at_zero_heading` | `heading=0`（正东）时，纯右侧偏置（`slot.z`，正值=右）映射为**负**的北向偏置（面向正东时右手边是正南），符号搞反会让编队槽位左右镜像 |
| `test_rotates_forward_offset_with_heading` | `heading=90°`（正北）时，纯前向偏置旋转成北向偏置，验证旋转矩阵方向而不只测 `heading=0` 这一特例 |
| `test_looseScale_multiplies_horizontal_offset_only` | `looseScale` 线性放大水平偏置（east/north），但高度偏置（`slot.y`）保持固定、不随 scale 扩展 |
| `test_route_start_offset_carries_through` | 集结区起点 A 非原点时，M_i 在 A 的基础上叠加旋转/缩放后的偏置，而非忽略 A |
| `test_climbing_first_segment_still_uses_horizontal_rally_plane` | 第一航段存在非零爬升倾角时，仍只用首段水平航向生成 ENU 平面 M_i；`slot.y` 保持天向固定差，禁止误套三维 FUR 倾角旋转 |

| 测试名 | 断言 |
| ------ | ---- |
| `test_rally_join_pos_entry_point_lies_on_loiter_circle` | FLYING 阶段算出的切入点 T 落在盘旋圆上（到圆心距离等于 `loiter_radius_m`），且一般不等于 `loose_slot` |
| `test_rally_join_pos_flying_to_loitering_transition_heading_jump_is_small` | 回归用例：即便 `arrival_radius_m` 配置得较大（如 100m），FLYING→LOITERING 切换瞬间的指令航向跳变也被内部夹到较小量级（<10°），不会随配置值线性放大（旧版 100m 时实测 ~26°） |
| `test_rally_join_pos_heading_jump_bound_holds_across_loiter_radii` | 回归用例：跳变角上限按 `loiter_radius_m` 反解触发半径（ψ=atan(d/R)），合法半径范围（200/100/50m）内跳变角都保持较小量级，不随 R 变小显著放大（旧版固定 15m 触发半径时 R=10m 能到 ~56°） |
| `test_rally_join_pos_rejects_loiter_radius_too_small_for_capture_window` | `loiter_radius_m` 太小（如 10m）时 `init()` 显式拒绝，而不是静默产出远超 5° 承诺的跳变角，或让触发窗口比每步飞行距离还窄而错过切入 |
| `test_rally_join_pos_rejects_arrival_radius_too_small_even_with_valid_loiter_radius` | 回归用例：即便 `loiter_radius_m` 合法，`arrival_radius_m` 单独配置得过小（如 1m）时 `init()` 仍显式拒绝——`min(arrival_radius_m, _arc_capture_radius_m)` 里 `arrival_radius_m` 可能是生效的更小值，只校验 `loiter_radius_m` 会漏判 |
| `test_rally_join_pos_capture_window_uses_worst_case_of_approach_and_loiter_min_speed` | 回归用例：`required_capture_radius_m` 按 `max(approach_speed_mps, loiter_speed_min_mps)` 取更快一侧的速度反解单步安全边界，不能只看 `approach_speed_mps`（LOITERING 圆弧巡航速度若显著更快，用较慢的 approach 速度反解会算出偏小的安全边界） |
| `test_rally_join_pos_exit_heading_matches_mission_heading_from_opposite_arrival` | 回归用例：即便从任务航向正对侧飞来（旧版会导致切出反向），切出速度方向仍精确等于 `mission_heading_rad` |
| `test_rally_join_pos_does_not_exit_immediately_when_entry_point_lands_just_past_slot_angle` | 回归用例：切入点 T 弦长离 M_i 很近但真实 CCW 弧长约 350°（T 在角度上"刚越过" M_i）时，不会被对称弧距 `ang_dist` 误判成"已到达"而跳过圆弧直接切出 |
| `test_rally_join_pos_does_not_exit_when_entry_point_lands_just_past_slot_angle` | 同一个“弦长近、真实弧长约 350°”场景下，进入 LOITERING 首拍不得被对称弧距误判为切出 |
| `test_rally_join_pos_loitering_targets_nominal_radius_not_actual` | LOITERING 阶段的位置指令/向心前馈按期望半径 `loiter_radius_m` 给出，不跟随飞机此刻的实际半径漂移 |
| `test_rally_join_pos_falls_back_to_direct_flight_when_starting_inside_circle` | 已知限制：起点落在盘旋圆内部/圆上（无切线可求）时退化为直飞 `loose_slot`，不抛异常 |

## 10. TestSlotGeometry - 最终槽位几何

| 测试名 | 断言 |
| ------ | ---- |
| `test_slot_geometry_position_and_velocity` | 位置和速度使用固定的最终槽位偏置 |
| `test_turn_feedforward_uses_slot_offset` | 长机 `dVPsi!=0` 时，刚体旋转速度使用最终槽位偏置 |
| `test_slot_geometry_up_normal_offset_in_level_flight` | 平飞时上法向与天向重合；爬升时不作“固定世界高度差”推论 |
| `test_slot_geometry_uses_full_fur_basis_while_leader_climbs` | 非零爬升角下 `x/y` 随倾角旋转，`z` 保持水平右侧向 |
| `test_slot_geometry_yaw_velocity_uses_fur_right_axis_and_mirrors_turn` | 三维偏航刚体速度符合 FUR 公式，左右转严格镜像 |
| `test_slot_geometry_td_seed_projects_full_enu_offset_to_fur` | TD 首拍把当前三维 ENU 相对位置反投影到 FUR，避免坐标轴切换阶跃 |
| `test_velocity_scalar_and_heading_are_recomputed` | 输出 `vd=hypot(vEast,vNorth)`，`vPsi=atan2(vNorth,vEast)` |
| `test_undefined_leader_track_falls_back_consistently` | 长机水平速度为 0 时，按现有 `SlotGeometry` 的东向兜底策略计算 |
| `test_missing_pattern_or_slot_raises` | 未知 `pattern` 或找不到本机槽位时抛 `ValueError` |

---

## 11/12. TestRallyEntity - 集结长机/僚机实体主链路

> 本节原分为 TestLeaderEntity/TestFollowerEntity 两节，旧版描述的是 `RallyApproach` 直飞 M_i、到达即锁存的流程测试点，随
> `RallyJoinPos`（切线进圆）重构已全部替换。当前长机/僚机实体级测试合并为同一个 `RallyEntityTests`
> 类（`tests/llt/test_formation_rally.py`），下表按实际用例重写。

| 测试名 | 断言 |
| ------ | ---- |
| `test_rally_follower_latches_arrival_and_switches_to_slot_after_step_one` | 僚机切出后上报 `rally_state=EXITED`；长机推进到 CATCHUP 后改用 `SlotGeometry` 计算最终槽位 |
| `test_rally_follower_waits_when_t_ref_is_not_valid_at_cold_start` | 冷启动尚无有效 T_ref 时，僚机进入 `LOITERING` 而非直接 `EXITED` |
| `test_rally_follower_none_resets_join_state_for_restart` | 已 `EXITED` 的僚机收到 `stage=NONE` 后，`RallyJoinPos` 复位回 `FLYING` |
| `test_rally_follower_none_outputs_current_position_zero_velocity` | `stage=NONE` 时 `selfCmd.pos` 复制本机当前位置、`selfCmd.v` 为零速 |
| `test_rally_leader_outputs_completion_event_once` | 长机汇合子状态置 `EXITED` 后推进多帧，LOOSE 收敛后直接进入 HOLD；`rallyCompleted` 只在转换首帧为 True |
| `test_rally_leader_none_resets_join_state` | 长机处于 `HOLD` 且已 `EXITED` 时收到 `remote=NONE`，`RallyJoinPos` 复位回 `FLYING` |
| `test_rally_leader_init_rejects_empty_route_list` | `route=[]`（空列表而非 `None`）时 `init()` 显式抛 `ValueError`，不应因空列表索引抛 `IndexError` |
| `test_route_heading_rejects_horizontally_degenerate_first_segment` | 回归用例：`route_heading_rad()` 遇到 A/A1 水平坐标重合（仅高度不同也算）时显式抛 `ValueError`，不静默按 `atan2(0,0)` 退化为正东 |
| `test_rally_leader_init_rejects_horizontally_degenerate_route` | 长机 `init()` 同样拒绝水平退化的统一 `route` 第一航段，而不是算出错误航向静默通过 |

---

## 11. TestRallySimControlIntegration - 仿真控制接入

| 测试名 | 断言 |
| ------ | ---- |
| `test_repository_rally_demo_5_aircraft_config_loads` | `configs/rally_demo_5_aircraft.json` 存在且能被 `sim_control.load_config()` 正常解析，旧三机 `configs/rally_demo.json` 不再保留 |
| `test_config_loader_rejects_removed_rally_route_fields` | 配置出现已移除的独立集结航线字段时明确报错，并提示统一使用 `route_file` |
| `test_rally_roles_select_rally_entities` | `role="rally_leader"` 创建 `LeaderEntity`，`role="rally_follower"` 创建 `FollowerEntity` |
| `test_start_rally_first_tick_prime_does_not_advance_ordinary_nodes_or_communication` | `leader/wingman` 与集结角色共用通用实体，集结首拍预热不推进普通保持节点或通信 |
| `test_rally_config_builds_expected_follower_ids` | `rally.expected_follower_ids` 注入 `RallyTaskInitS.expectedFollowerIds` |
| `test_validate_accepts_rally_roles_with_route_only` | 集结角色只配置统一 `route` 时通过校验，首点作为集结中心、首段作为集结方向 |
| `test_validate_rejects_rally_leader_without_route` | 集结角色缺少统一 `route` 时配置校验失败 |
| `test_validate_rejects_loiter_radius_too_small_for_capture_window` | `validate()` 在配置加载阶段就调用 `validate_capture_geometry()`（与 `RallyJoinPos.init()` 复用同一份逻辑），`loiter_radius_m` 太小时直接拒绝，不必等到实体构造才失败 |
| `test_rally_remote_defaults_to_rally_until_completion` | 集结场景运行时 `_NodeAlgorithm.step` 下发 `RemoteCmdS(FormStageE.RALLY)` |
| `test_hold_scene_remote_remains_hold` | 非集结场景仍下发 `RemoteCmdS(FormStageE.HOLD)` |
| `test_now_s_is_injected_to_entity_input` | `_NodeAlgorithm.step(..., time_s=t)` 构造 `EntityInputS(now_s=t)` |
| `test_rally_leader_broadcast_reaches_rally_followers` | 通过通信通道，长机阶段、队形和公共计划广播到僚机 inbox 并被解析 |
| `test_rally_status_reaches_leader` | 僚机 `formation.follower_status` 通过通信通道到达长机并更新 `followerStates` |
| `test_snapshot_excludes_removed_rally_analysis` | 编队分析删除后，控制器和仿真快照均不再暴露分析字段 |
| `test_remote_stage_switches_to_hold_after_completion` | 集结完成后控制器自动将 `_remote_stage` 切为 `HOLD`，后续帧 `EntityInputS.remote.stage == HOLD` |

---

## 12. TestRallyEndToEndScenario - 小规模闭环场景

| 测试名 | 断言 |
| ------ | ---- |
| `test_two_followers_reach_hold_after_rally` | 1 长机 + 2 僚机配置，运行足够时长后长机 `cmd.stage==HOLD` |
| `test_followers_first_report_exited_then_slot_error_converges` | 僚机先广播 `rally_state=EXITED`，之后 `posErr_m` 进入松散/紧密阈值 |
| `test_link_loss_freezes_rally_progress` | 某期望僚机链路丢失超过 `staleTimeout_s` 时，Rally 不推进到下一阶段 |
| `test_recover_after_link_loss_allows_progress` | 链路恢复并重新收到有效状态后，状态机继续推进 |
| `test_reset_restarts_rally_from_approach` | 集结运行中调用 `reset()`，仿真复位且 `_remote_stage` 重置为 `RALLY`，下一帧长机输出 `cmd.stage=RALLY/step=0`（NONE→RALLY 重启语义已在 TestRallyTaskRemote.test_none_then_rally_allows_restart 单元级覆盖） |

---

## 覆盖矩阵

| LLD 需求 | 覆盖测试类 |
| -------- | ---------- |
| 新增叶类型、copy 函数、Context reset | TestRallyLeafTypesAndContext |
| EntityInputS/EntityInitS/EntityOutputS 扩展 | TestEntityBoundaryTypes |
| Rally 参数校验与 reset | TestRallyTaskValidation |
| remote NONE/HOLD/RALLY 语义 | TestRallyTaskRemote |
| JOINING/CATCHUP/LOOSE/HOLD 状态流转 | TestRallyTask |
| expectedFollowerIds、lastUpdate_s、超时冻结 | TestRallyTask, TestFollowerStatusInbound |
| `cmd.pattern` 首拍写入 targetPattern | TestRallyTask |
| 僚机状态广播与锁存 rally_state/误差/基础航程 | TestFollowerBroadcast, TestRallyEntity |
| 长机解析僚机回报 | TestFollowerStatusInbound |
| 统一长机广播携带阶段、队形与公共计划 | TestRallyLeaderBroadcastAndInbound |
| RallyJoinPos 切入盘旋圆、圆弧汇合、切出航向 | TestRallyJoinPos |
| `rally_loose_target()` ENU 水平集结几何、右侧轴符号、缩放/高度及非零倾角边界 | TestRallyLooseTarget |
| `loiter_speed_bounds()` 上下限推导与序校验 | TestRallyLoiterSpeedBounds |
| SlotGeometry 最终槽位与转弯前馈 | TestSlotGeometry |
| LeaderEntity/FollowerEntity 主链路（JOINING→CATCHUP/LOOSE→HOLD、NONE 复位） | TestRallyEntity |
| `rallyCompleted` 只在自然完成转换拍置位一次 | TestRallyEntity |
| 配置解析、角色映射、now_s 注入、remote=RALLY 接入、完成后自动切 HOLD、锁存清空 | TestRallySimControlIntegration |
| 多机闭环集结、断链冻结、恢复继续、重启 | TestRallyEndToEndScenario |

---

## 执行命令

```bash
python -m unittest tests.llt.test_formation_rally tests.llt.test_sim_control_rally
```

提交前仍需运行项目基础检查：

```bash
python -m compileall -q src
git diff --check
```

若本轮实现修改了 `src/` 下 Python 代码，还需执行注释覆盖率检查：

```bash
python -X utf8 scripts/comment_coverage.py \
  --fail-under-module 100 \
  --fail-under-class 100 \
  --fail-under-func 100 \
  --fail-under-inline 15 \
  --worst 12
```
