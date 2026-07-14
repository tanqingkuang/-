"""Simulation control facade.

The controller implements the application contract described in
``docs/1-仿真控制HLD.md``. This module remains the compatibility import point;
large implementation sections live in neighboring focused modules.
"""

from __future__ import annotations

import time

from src.runner.sim_control_constants import (
    _COMM_DECIMATION,
    _CPU_UTILIZATION_SAMPLE_PERIOD_S,
    _DEFAULT_ALGORITHM_DECIMATION,
    _DEFAULT_TRIANGLE_WING_SLOTS,
    _FORMATION_COORDINATE_SYSTEM,
    _LOG_SAMPLE_PERIOD_S,
    _MAX_PLAYBACK_RATE,
    _MAX_RUN_LOOP_BATCH_TICKS,
    _MIN_PLAYBACK_RATE,
    _RUN_LOOP_SLEEP_SLICE_S,
    _TIME_EPSILON_S,
)
from src.runner.sim_control_modules import _ConfigLoader, _DataLogger, _DisturbanceEngine, _NodeAlgorithm
from src.runner.sim_control_routes import (
    _build_formation_comm_init,
    _build_formation_slots,
    _build_leader_route,
    _build_rally_approach_speed,
    _build_rally_task_init,
    _build_vel_cmd_limit,
    _default_formation_slots,
    _default_leader_wpi,
    _float_from_keys,
    _leader_id_from_nodes,
    _motion_from_aircraft_state,
    _route_point_from_config,
    _route_state_from_wayline,
    _same_xy,
    _validate_formation_coordinate_system,
    _wayline_from_config,
    _waypoint_center,
    _waypoint_radius,
    _waypoint_turn_sign,
    _wpi_from_waypoints,
)
from src.runner.sim_control_types import (
    CommandResult,
    ControlReport,
    DisturbanceCommand,
    DisturbanceType,
    EventLevel,
    LinkState,
    NodeState,
    RallyPlanGeometryState,
    ResultCode,
    RouteState,
    RunState,
    SimulationEvent,
    SimulationSnapshot,
    Subscription,
    TimedSnapshotCursor,
    _ConfiguredLink,
    _NodeAlgorithmOutput,
)
from src.runner.gui_application import (
    AvoidanceParams,
    AvoidancePlanOutcome,
    GeoReference,
    GuiConfigData,
    ObstacleKind,
    ObstacleSpec,
    PlannedRoute,
    apply_planned_route,
    export_planned_route,
    geodetic_from_enu,
    load_gui_config,
    persist_config_duration,
    plan_route_for_gui,
    planned_route_from_waypoints,
    preview_route_marker_points,
    route_export_defaults,
    route_inputs_to_config,
    route_to_polyline,
)
from src.runner.sim_controller import SimulationController
