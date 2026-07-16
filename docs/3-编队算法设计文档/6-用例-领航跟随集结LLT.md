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
- `src/algorithm/units/algo/pos_calc/slot_geometry.py`（CATCHUP/LOOSE/COMPRESS 共用；原 `catchup_align.py` 已删除）
- `src/algorithm/entity/leader_follower_rally/leader.py`
- `src/algorithm/entity/leader_follower_rally/follower.py`
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
    arrived: int = 0,
    valid: bool = True,
    last_update_s: float = 0.0,
    rally_state: str = "EXITED",
    eta_s: float = 0.0,
    reached_slot_once: bool = False,
) -> FollowerStateS:
    return FollowerStateS(
        id=node_id, pos=PosInEarthS(), posErr_m=pos_err_m, arrived=arrived, valid=valid,
        lastUpdate_s=last_update_s, rally_state=rally_state, eta_s=eta_s, reachedSlotOnce=reached_slot_once,
    )


def _follower_status_msg(
    source: str = "R02",
    *,
    pos_east: float = 0.0,
    pos_north: float = 0.0,
    pos_h: float = 500.0,
    pos_err_m: float = 0.0,
    arrived: int = 0,
    rally_state: str = "EXITED",
    eta_s: float = 0.0,
) -> MessageEnvelope:
    return MessageEnvelope(
        topic=FOLLOWER_STATUS_TOPIC, source=source, target="R01", timestamp=0.0,
        payload={
            "id": source, "pos_east": pos_east, "pos_north": pos_north, "pos_h": pos_h,
            "pos_err_m": pos_err_m, "arrived": arrived, "rally_state": rally_state, "eta_s": eta_s,
        },
    )


def _leader_msg(
    *,
    stage: FormStageE = FormStageE.RALLY,
    pattern: int = 0,
    step: int = 0,
    scale: float = 3.0,
    scale_rate: float = 0.0,
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
            "slot_scale": {"scale": scale, "scale_rate": scale_rate},
            "t_ref": t_ref,
            "t_ref_valid": t_ref_valid,
        },
    )


def _rally_task(
    expected: tuple[str, ...] = ("R02", "R03"),
    *,
    dt_s: float = 0.1,
    stable_hold_s: float = 0.2,
    compress_time_s: float = 1.0,
    catchup_radius_m: float = 200.0,
    catchup_stable_s: float = 0.0,
) -> Rally:
    task = Rally()
    task.init(
        RallyTaskInitS(
            looseScale=3.0, convergenceRadius_m=5.0, stableHold_s=stable_hold_s,
            compressTime_s=compress_time_s, tightRadius_m=2.0, expectedFollowerIds=list(expected),
            staleTimeout_s=0.5, targetPattern=0, dt_s=dt_s,
            catchup_radius_m=catchup_radius_m, catchup_stable_s=catchup_stable_s,
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
    leader_join_exited: bool = True,
    leader_join_flying: bool = False,
    leader_join_reached_slot_once: bool = False,
    leader_eta_s: float = 0.0,
) -> RallyTaskOutputS:
    """推进 Rally 任务一拍并返回输出端口。"""
    output = RallyTaskOutputS(cmd=ctx.cmd, slotScale=ctx.slotScale)
    task.step(
        RallyTaskInputS(
            remote=RemoteCmdS(remote), cmd=ctx.cmd, followerStates=states or [], now_s=now_s,
            leader_join_exited=leader_join_exited, leader_join_flying=leader_join_flying,
            leader_join_reached_slot_once=leader_join_reached_slot_once, leader_eta_s=leader_eta_s,
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
        WayPointInputS(idx=0, pos=PosInEarthS(*start), vdCmd=speed),
        WayPointInputS(idx=1, pos=PosInEarthS(*end), vdCmd=speed),
    ]


def _rally_cfg(
    *,
    expected: tuple[str, ...] = ("R02", "R03"),
    dt_s: float = 0.1,
    stable_hold_s: float = 0.1,
    compress_time_s: float = 0.1,
    catchup_stable_s: float = 0.0,
) -> RallyTaskInitS:
    """构造实体测试用集结配置。"""
    return RallyTaskInitS(
        looseScale=3.0, convergenceRadius_m=5.0, stableHold_s=stable_hold_s,
        compressTime_s=compress_time_s, tightRadius_m=2.0, expectedFollowerIds=list(expected),
        staleTimeout_s=1.0, targetPattern=0, dt_s=dt_s, catchup_stable_s=catchup_stable_s,
    )
```

---

## 1. TestRallyLeafTypesAndContext - 叶类型与 Context

| 测试名 | 断言 |
| ------ | ---- |
| `test_rally_slot_scale_defaults_to_final_scale` | `RallySlotScaleS()` 默认 `scale==1.0`、`scaleRate==0.0` |
| `test_follower_state_defaults_invalid` | `FollowerStateS()` 默认 `valid is False`、`arrived==0`、`lastUpdate_s==0.0` |
| `test_formation_analysis_defaults_zero` | `FormationAnalysisS()` 默认误差、数量字段为 0 |
| `test_copy_rally_slot_scale_copies_scale_and_rate` | `copy_rally_slot_scale` 同时复制 `scale` 和 `scaleRate` |
| `test_copy_follower_state_copies_valid_and_last_update` | `copy_follower_state` 复制 `id/pos/posErr_m/arrived/valid/lastUpdate_s` |
| `test_copy_formation_analysis_copies_all_fields` | `copy_formation_analysis` 覆盖所有诊断字段 |
| `test_context_contains_rally_fields` | `FormContextS` 含 `slotScale` 与 `followerStates`，且列表默认独立 |
| `test_reset_context_resets_slot_scale_and_clears_follower_states` | `reset_context` 后 `slotScale.scale==1.0`、`scaleRate==0.0`、`followerStates==[]` |

---

## 2. TestEntityBoundaryTypes - 实体边界扩展

| 测试名 | 断言 |
| ------ | ---- |
| `test_entity_input_contains_now_s_default_zero` | `EntityInputS().now_s == 0.0` |
| `test_entity_boundary_defaults_include_rally_fields` | `EntityInitS.route` 默认空列表且不再含 `rally_route`；M_i 由 init 按统一 `route` 前两点和队形槽位自动推导，`rally_cfg/rally_approach_speed_mps/rally_leader_id` 默认值符合 LLD（`rally_leader_id=""` 为默认值） |
| `test_entity_output_contains_optional_formation_analysis` | `EntityOutputS().formationAnalysis is None` |
| `test_existing_hold_entities_accept_extended_boundary_types` | 现有 `LeaderEntity/FollowerEntity` 使用扩展后的 `EntityInputS/OutputS` 构造并 step 不抛异常 |

---

## 3. TestRallyTaskValidation - Rally 初始化校验

| 测试名 | 断言 |
| ------ | ---- |
| `test_init_rejects_loose_scale_below_one` | `looseScale < 1.0` 抛 `ValueError` |
| `test_init_rejects_zero_compress_time` | `compressTime_s <= 0` 抛 `ValueError` |
| `test_init_rejects_zero_stale_timeout` | `staleTimeout_s <= 0` 抛 `ValueError` |
| `test_init_rejects_zero_dt` | `dt_s <= 0` 抛 `ValueError` |
| `test_reset_restores_none_and_loose_scale` | `reset()` 清零计时器，下一次 `remote=NONE` step 输出 `NONE/step=0/pattern=NONE/scale=looseScale` |

---

## 4. TestRallyTaskRemote - Rally 遥控语义

| 测试名 | 断言 |
| ------ | ---- |
| `test_remote_none_outputs_none_and_loose_scale` | `remote=NONE` 输出 `cmd.stage=NONE`、`pattern=NONE`、`slotScale.scale=looseScale` |
| `test_remote_none_resets_running_timers` | RALLY 中已累加计时器后切 `NONE`，再进 RALLY 不继承旧计时 |
| `test_remote_hold_outputs_hold_final_scale` | `remote=HOLD` 输出 `HOLD`、`targetPattern`、`slotScale.scale=1.0`、`scaleRate=0.0` |
| `test_remote_hold_does_not_mark_rally_completed` | 外部强制 HOLD 不设置正常完成标志，实体不应输出 `FormationAnalysisS` |
| `test_remote_rally_from_none_starts_approach` | `NONE + remote=RALLY` 重置计时器并输出 `RALLY/step=0/targetPattern` |
| `test_remote_rally_after_completed_hold_does_not_restart` | 已正常完成到 `HOLD` 后继续 `remote=RALLY`，保持 `HOLD`，不回到 `APPROACH` |
| `test_none_then_rally_allows_restart` | 完成后先发 `NONE` 再发 `RALLY`，允许重新从 `APPROACH` 开始 |

---

## 5. TestRallyTaskApproachLooseCompress - Rally 状态机

| 测试名 | 断言 |
| ------ | ---- |
| `test_empty_expected_followers_advances_by_timers` | `expectedFollowerIds=[]` 时，APPROACH/LOOSE 可按计时器立即推进 |
| `test_missing_follower_states_freezes_approach` | 期望列表非空但 `followerStates=[]`，保持 `APPROACH`，`_arrive_timer` 不累加 |
| `test_invalid_or_stale_follower_freezes_approach` | `valid=False` 或 `now_s-lastUpdate_s>staleTimeout_s`，不切 LOOSE |
| `test_arrived_flag_controls_approach_not_pos_error` | `arrived==1` 且 `posErr_m` 很大，APPROACH 仍可按到达锁存推进 |
| `test_arrive_timer_requires_continuous_arrived` | 连续满足到达才切 LOOSE，中间一帧未到达会清零计时器 |
| `test_first_rally_frame_writes_target_pattern` | 第一拍 `RALLY/APPROACH` 即写 `cmd.pattern=targetPattern` |
| `test_loose_uses_position_error_threshold` | LOOSE 阶段按 `posErr_m < convergenceRadius_m` 判定稳定 |
| `test_loose_timer_resets_when_any_expected_follower_outside_radius` | 任一期望僚机误差超阈值，`_stable_timer` 清零 |
| `test_loose_advances_to_compress_after_stable_hold` | 全部误差达标并持续 `stableHold_s` 后输出 `step=2` |
| `test_compress_scale_decreases_linearly` | COMPRESS 每拍 `scale` 按 `looseScale -> 1.0` 线性递减 |
| `test_compress_scale_rate_is_negative_until_final_scale` | `scale>1.0` 时 `scaleRate=-(looseScale-1)/compressTime_s` |
| `test_compress_scale_rate_zero_at_final_scale` | `scale==1.0` 后 `scaleRate==0.0` |
| `test_compress_waits_for_tight_error_after_scale_done` | `scale==1.0` 但任一僚机 `posErr_m>=tightRadius_m`，仍保持 `RALLY/step=2` |
| `test_compress_to_hold_sets_rally_completed_once` | `scale==1.0` 且误差达标时输出 `HOLD`，并只在转换拍输出 `rallyCompleted=True` |

---

## 6. TestFollowerBroadcast - 僚机状态广播

| 测试名 | 断言 |
| ------ | ---- |
| `test_targets_leader_from_cfg` | `cfg.leaderId` 非空时，`MessageEnvelope.target` 等于 `cfg.leaderId` |
| `test_broadcast_topic_is_follower_status` | 输出 topic 固定为 `formation.follower_status` |
| `test_payload_contains_position_error_and_arrived` | payload 含 `id/pos_east/pos_north/pos_h/pos_err_m/arrived` |
| `test_pos_error_is_distance_to_self_cmd` | `pos_err_m = norm(selfState.pos - selfCmd.pos)` |
| `test_arrived_uses_entity_latched_value` | `arrived` 直接等于 `u.selfArrived`，不由当前距离反算 |
| `test_empty_leader_id_raises_value_error` | `cfg.leaderId == ""` 时 `init()` 抛 `ValueError`（不依赖 netWork 推断） |
| `test_missing_ports_raise_value_error` | `selfState/selfCmd/outbox` 未绑定时抛 `ValueError` |

---

## 7. TestFollowerStatusInbound - 长机解析僚机回报

| 测试名 | 断言 |
| ------ | ---- |
| `test_parses_two_follower_messages` | 两条 `formation.follower_status` 写入两个 `FollowerStateS`，字段完整 |
| `test_updates_existing_entry_in_place` | 已有同 ID 条目时原地更新，不追加重复项 |
| `test_appends_new_follower_entry` | 新 source 追加到 `followerStates` |
| `test_sets_valid_and_last_update` | 收到报文后 `valid=True`、`lastUpdate_s=now_s` |
| `test_empty_inbox_keeps_last_update` | 断链帧不更新 `lastUpdate_s`，不清空旧状态 |
| `test_filters_non_follower_status_topics` | `formation.leader`、`node.status` 等非目标 topic 被忽略 |
| `test_ignores_non_dict_or_incomplete_payload` | payload 非 dict 或关键字段缺失时跳过，不写半截状态 |
| `test_payload_id_mismatch_uses_envelope_source` | payload 中 `id` 与 `envelope.source` 不一致时，以 `envelope.source` 作为 `FollowerStateS.id`，不信任 payload id |

---

## 8. TestRallyLeaderBroadcastAndInbound - 统一长机广播与僚机解析

| 测试名 | 断言 |
| ------ | ---- |
| `test_rally_leader_broadcast_keeps_existing_payload_contract` | payload 保留 `leader_state` 与 `cmd`，topic 仍为 `formation.leader` |
| `test_rally_leader_broadcast_adds_slot_scale` | payload 包含 `slot_scale.scale` 与 `slot_scale.scale_rate`，保持场景默认 scale=1.0 |
| `test_rally_leader_follower_parses_cmd_leader_state_and_slot_scale` | 入站单元同时写入 `leaderState/leaderCmd/cmd/slotScale`，缺少 `cmd["leader"]` 时 `leaderCmd` 回退为 `leaderState` |
| `test_missing_slot_scale_defaults_final_scale` | 老格式长机广播无 `slot_scale` 时，解析为 `scale=1.0`、`scaleRate=0.0` |
| `test_malformed_slot_scale_defaults_final_scale` | `slot_scale` 非 dict 或字段不可转 float 时解析为 `scale=1.0`、`scaleRate=0.0` |
| `test_non_leader_topic_is_skipped` | 非 `formation.leader` topic 不改写 `leaderState/leaderCmd/cmd/slotScale` |
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
槽位；进入 CATCHUP/LOOSE/COMPRESS 后由 `SlotGeometry` 接管三维变换：

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
| `test_rally_join_pos_reached_slot_once_stays_false_when_entry_point_lands_just_past_slot_angle` | 回归用例：同一个"弦长近、真实弧长约 350°"场景下，进 LOITERING 那一拍 `reached_slot_once` 必须仍是 False，不能被对称弧距误判成"已到达"（复现过一次：`_step_loitering` 里独立的 `ang_dist` 判断会把 `_enter_arc` 刚设对的 False 立刻翻回 True，修复为复用 `_away_from_slot` 同一判据） |
| `test_rally_join_pos_loitering_targets_nominal_radius_not_actual` | LOITERING 阶段的位置指令/向心前馈按期望半径 `loiter_radius_m` 给出，不跟随飞机此刻的实际半径漂移 |
| `test_rally_join_pos_falls_back_to_direct_flight_when_starting_inside_circle` | 已知限制：起点落在盘旋圆内部/圆上（无切线可求）时退化为直飞 `loose_slot`，不抛异常 |

**T_ref 聚合新增回归用例**（`reached_slot_once`，修复"到达切入点 T 就过早退出 T_ref 聚合"问题，位于
`tests/llt/test_formation_rally.py::RallyTaskTests`，与本节其余 `RallyJoinPos` 单测不同类）：

| 测试名 | 断言 |
| ------ | ---- |
| `test_t_ref_counts_loitering_follower_that_has_not_reached_slot_once` | LOITERING 但尚未首次路过 M_i（`reachedSlotOnce=False`）的僚机 ETA 仍计入 T_ref，不因状态从 FLYING 变成 LOITERING 就被剔除 |
| `test_t_ref_excludes_loitering_follower_after_reaching_slot_once` | 已首次路过 M_i（`reachedSlotOnce=True`）、纯粹盘旋等待的僚机不再计入 T_ref，避免其每圈波动的 ETA 反复推高/拉低基准时间 |

---

## 10. TestSlotGeometry - 带缩放槽位几何

| 测试名 | 断言 |
| ------ | ---- |
| `test_scale_one_matches_slot_geometry_position_and_velocity` | `scale=1.0/scaleRate=0` 时，位置和速度与现有 `SlotGeometry` 一致 |
| `test_scale_two_doubles_position_offset` | 平飞时 `scale=2.0` 使 FUR 的 `x/z` 偏置扩大 2 倍，`y` 不变 |
| `test_scale_rate_adds_compression_velocity` | `scaleRate<0` 时速度多出由 FUR 的 `x/z` 映射到 ENU 的压缩分量 |
| `test_turn_feedforward_uses_scaled_offset` | 长机 `dVPsi!=0` 时，刚体旋转速度使用 `scale * slot.x/z` |
| `test_slot_geometry_up_normal_offset_not_scaled_in_level_flight` | 平飞时上法向与天向重合，`slot.y` 不随 `scale` 放大；爬升时不作“固定世界高度差”推论 |
| `test_slot_geometry_uses_full_fur_basis_while_leader_climbs` | 非零爬升角下 `x/y` 随倾角旋转，`z` 保持水平右侧向 |
| `test_slot_geometry_yaw_velocity_uses_fur_right_axis_and_mirrors_turn` | 三维偏航刚体速度符合 FUR 公式，左右转严格镜像 |
| `test_slot_geometry_td_seed_projects_full_enu_offset_to_fur` | TD 首拍把当前三维 ENU 相对位置反投影到 FUR，避免坐标轴切换阶跃 |
| `test_velocity_scalar_and_heading_are_recomputed` | 输出 `vd=hypot(vEast,vNorth)`，`vPsi=atan2(vNorth,vEast)` |
| `test_undefined_leader_track_falls_back_consistently` | 长机水平速度为 0 时，按现有 `SlotGeometry` 的东向兜底策略计算 |
| `test_missing_pattern_or_slot_raises` | 未知 `pattern` 或找不到本机槽位时抛 `ValueError` |

---

## 11/12. TestRallyEntity - 集结长机/僚机实体主链路

> 本节原分为 TestRallyLeaderEntity/TestRallyFollowerEntity 两节，列出的测试名（`test_rally_before_arrival_uses_rally_approach`、
> `_self_arrived`、`_rally_target` 等）描述的是 `RallyApproach` 直飞 M_i、到达即锁存的旧流程测试点，随
> `RallyJoinPos`（切线进圆）重构已全部替换。当前长机/僚机实体级测试合并为同一个 `RallyEntityTests`
> 类（`tests/llt/test_formation_rally.py`），下表按实际用例重写。

| 测试名 | 断言 |
| ------ | ---- |
| `test_rally_follower_latches_arrival_and_switches_to_slot_scale_after_step_one` | 僚机位于目标点且 T_ref 已有效时 `RallyJoinPos` 进入 `EXITED`，上报 `arrived=1`/`rally_state=EXITED`；长机推进到 `step=1`（CATCHUP）后 `selfCmd.pos` 变为 `SlotGeometry` 按长机状态算出的真实缩放槽位（与 LOOSE 同一算法） |
| `test_rally_follower_waits_when_t_ref_is_not_valid_at_cold_start` | 冷启动尚无有效 T_ref（`t_ref_valid=False`）时，即便已到目标点附近，`RallyJoinPos` 进入 `LOITERING` 而非直接 `EXITED`，上报 `arrived=0` |
| `test_rally_follower_none_resets_join_state_for_restart` | 已 `EXITED` 的僚机收到 `stage=NONE` 后，`RallyJoinPos` 复位回 `FLYING`，上报 `arrived=0`，允许下一轮重新执行 JOINING |
| `test_rally_follower_none_outputs_current_position_zero_velocity` | `stage=NONE` 时 `selfCmd.pos` 复制本机当前位置、`selfCmd.v` 为零速，上报 `arrived=0` |
| `test_rally_leader_completes_and_outputs_formation_analysis_once` | 长机汇合子状态置 `EXITED` 后推进多帧，COMPRESS 正常完成分析 `formationAnalysis` 只在首帧非 None，其余帧为 None，字段值（`posErrMax_m/posErrRms_m/inPositionCount/totalCount`）与僚机回报一致 |
| `test_rally_leader_none_resets_join_and_completion_latches` | 长机处于 `HOLD` 且已 `EXITED`/`_rally_completed=True` 时收到 `remote=NONE`，`RallyJoinPos` 复位回 `FLYING`，`_rally_completed` 复位为 `False` |
| `test_rally_leader_init_rejects_empty_route_list` | `route=[]`（空列表而非 `None`）时 `init()` 显式抛 `ValueError`，不应因空列表索引抛 `IndexError` |
| `test_route_heading_rejects_horizontally_degenerate_first_segment` | 回归用例：`route_heading_rad()` 遇到 A/A1 水平坐标重合（仅高度不同也算）时显式抛 `ValueError`，不静默按 `atan2(0,0)` 退化为正东 |
| `test_rally_leader_init_rejects_horizontally_degenerate_route` | 长机 `init()` 同样拒绝水平退化的统一 `route` 第一航段，而不是算出错误航向静默通过 |

---

## 11. TestRallySimControlIntegration - 仿真控制接入

| 测试名 | 断言 |
| ------ | ---- |
| `test_repository_rally_demo_5_aircraft_config_loads` | `configs/rally_demo_5_aircraft.json` 存在且能被 `sim_control.load_config()` 正常解析，旧三机 `configs/rally_demo.json` 不再保留 |
| `test_config_loader_rejects_removed_rally_route_fields` | 配置出现已移除的独立集结航线字段时明确报错，并提示统一使用 `route_file` |
| `test_rally_roles_select_rally_entities` | `role="rally_leader"` 创建 `RallyLeaderEntity`，`role="rally_follower"` 创建 `RallyFollowerEntity` |
| `test_legacy_roles_still_select_hold_entities` | `leader/wingman` 角色仍创建现有 hold 实体，保持既有场景兼容 |
| `test_rally_config_builds_expected_follower_ids` | `rally.expected_follower_ids` 注入 `RallyTaskInitS.expectedFollowerIds` |
| `test_validate_accepts_rally_roles_with_route_only` | 集结角色只配置统一 `route` 时通过校验，首点作为集结中心、首段作为集结方向 |
| `test_validate_rejects_rally_leader_without_route` | 集结角色缺少统一 `route` 时配置校验失败 |
| `test_validate_rejects_loiter_radius_too_small_for_capture_window` | `validate()` 在配置加载阶段就调用 `validate_capture_geometry()`（与 `RallyJoinPos.init()` 复用同一份逻辑），`loiter_radius_m` 太小时直接拒绝，不必等到实体构造才失败 |
| `test_rally_remote_defaults_to_rally_until_completion` | 集结场景运行时 `_NodeAlgorithm.step` 下发 `RemoteCmdS(FormStageE.RALLY)` |
| `test_hold_scene_remote_remains_hold` | 非集结场景仍下发 `RemoteCmdS(FormStageE.HOLD)` |
| `test_now_s_is_injected_to_entity_input` | `_NodeAlgorithm.step(..., time_s=t)` 构造 `EntityInputS(now_s=t)` |
| `test_rally_leader_broadcast_reaches_rally_followers` | 通过通信通道，长机 `slot_scale` 广播到僚机 inbox 并被解析 |
| `test_rally_status_reaches_leader` | 僚机 `formation.follower_status` 通过通信通道到达长机并更新 `followerStates` |
| `test_rally_snapshot_exposes_formation_completed_analysis_when_complete` | 正常完成集结后，仿真快照字段 `formation_completed_analysis` 携带 `FormationAnalysisS` 诊断结果 |
| `test_formation_completed_analysis_cleared_on_load_and_reset` | `load_config()` 和 `reset()` 后快照 `formation_completed_analysis is None`，不携带上次集结结果 |
| `test_remote_stage_switches_to_hold_after_completion` | 集结完成后控制器自动将 `_remote_stage` 切为 `HOLD`，后续帧 `EntityInputS.remote.stage == HOLD` |

---

## 12. TestRallyEndToEndScenario - 小规模闭环场景

| 测试名 | 断言 |
| ------ | ---- |
| `test_two_followers_reach_hold_after_rally` | 1 长机 + 2 僚机配置，运行足够时长后长机 `cmd.stage==HOLD` |
| `test_followers_first_report_arrived_then_slot_error_converges` | 僚机先广播 `arrived=1`，之后 `posErr_m` 进入松散/紧密阈值 |
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
| APPROACH/LOOSE/COMPRESS/HOLD 状态流转 | TestRallyTaskApproachLooseCompress |
| expectedFollowerIds、valid、lastUpdate_s、超时冻结 | TestRallyTaskApproachLooseCompress, TestFollowerStatusInbound |
| `cmd.pattern` 首拍写入 targetPattern | TestRallyTaskApproachLooseCompress |
| `slotScale.scale/scaleRate` 输出 | TestRallyTaskApproachLooseCompress, TestSlotGeometry |
| 僚机状态广播与锁存 arrived/rally_state/eta_s/reached_slot_once | TestFollowerBroadcast, TestRallyEntity |
| 长机解析僚机回报 | TestFollowerStatusInbound |
| 统一长机广播保持既有 payload 并携带默认/动态 `slot_scale` | TestRallyLeaderBroadcastAndInbound |
| 僚机解析长机 `slot_scale`，缺字段默认 | TestRallyLeaderBroadcastAndInbound |
| RallyJoinPos 切入盘旋圆、圆弧汇合、切出航向 | TestRallyJoinPos |
| `rally_loose_target()` ENU 水平集结几何、右侧轴符号、缩放/高度及非零倾角边界 | TestRallyLooseTarget |
| `loiter_speed_bounds()` 上下限推导与序校验 | TestRallyLoiterSpeedBounds |
| SlotGeometry 缩放、压缩速度前馈、转弯前馈 | TestSlotGeometry |
| RallyLeaderEntity/RallyFollowerEntity 主链路（JOINING→CATCHUP/LOOSE/COMPRESS、NONE 复位） | TestRallyEntity |
| FormationAnalysis 只在正常完成后输出一次 | TestRallyEntity |
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
