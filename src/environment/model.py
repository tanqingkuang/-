"""Three-degree-of-freedom UAV point-mass model in ENU coordinates."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace


STATE_SIZE = 12
_COS_THETA_EPS = 1e-3
_MAX_ABS_THETA_RAD = math.radians(89.0)


@dataclass(frozen=True)
class AccelerationCommand:
    """ENU acceleration command produced by the node controller.

    ``ax`` points east, ``ay`` points north, and ``az`` points up.
    """

    ax_cmd_mps2: float = 0.0
    ay_cmd_mps2: float = 0.0
    az_cmd_mps2: float = 0.0

    def as_vector(self) -> tuple[float, float, float]:
        return (self.ax_cmd_mps2, self.ay_cmd_mps2, self.az_cmd_mps2)


@dataclass(frozen=True)
class PointMassModelConfig:
    """Numerical and physical constants for the 3-DOF point-mass model."""

    gravity_mps2: float
    min_speed_mps: float
    natural_frequency_rad_s: float
    damping_ratio: float
    acceleration_command_limit_mps2: float
    nx_min: float
    nx_max: float
    nz_min: float
    nz_max: float
    phi_min_rad: float
    phi_max_rad: float


@dataclass
class AircraftState:
    """Runtime state of one UAV.

    The augmented state vector is ordered as
    ``[E, N, U, V, theta, psi, aE, aN, aU, aE_dot, aN_dot, aU_dot]``.
    The last six states are the second-order response to the ENU
    acceleration command. They are converted to ``N_x / N_z / phi`` before
    evaluating the point-mass equations.
    """

    node_id: str
    role: str
    health: str
    x_m: float
    y_m: float
    altitude_m: float
    speed_mps: float
    theta_rad: float
    psi_rad: float
    ax_mps2: float
    ay_mps2: float
    az_mps2: float
    ax_rate_mps3: float
    ay_rate_mps3: float
    az_rate_mps3: float
    nx: float
    nz: float
    phi_rad: float
    cross_track_error_m: float
    distance_to_go_m: float

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)

    @property
    def psi_v_deg(self) -> float:
        return math.degrees(self.psi_rad)

    @property
    def phi_deg(self) -> float:
        return math.degrees(self.phi_rad)

    @property
    def vx_mps(self) -> float:
        return self.speed_mps * math.cos(self.theta_rad) * math.cos(self.psi_rad)

    @property
    def vy_mps(self) -> float:
        return self.speed_mps * math.cos(self.theta_rad) * math.sin(self.psi_rad)

    @property
    def vz_mps(self) -> float:
        return self.speed_mps * math.sin(self.theta_rad)

    def as_vector(self) -> list[float]:
        return [
            self.x_m,
            self.y_m,
            self.altitude_m,
            self.speed_mps,
            self.theta_rad,
            self.psi_rad,
            self.ax_mps2,
            self.ay_mps2,
            self.az_mps2,
            self.ax_rate_mps3,
            self.ay_rate_mps3,
            self.az_rate_mps3,
        ]

    def update_from_vector(self, vector: Sequence[float]) -> None:
        (
            self.x_m,
            self.y_m,
            self.altitude_m,
            self.speed_mps,
            self.theta_rad,
            self.psi_rad,
            self.ax_mps2,
            self.ay_mps2,
            self.az_mps2,
            self.ax_rate_mps3,
            self.ay_rate_mps3,
            self.az_rate_mps3,
        ) = vector


@dataclass(frozen=True)
class PointMassInputs:
    """Inputs to the 3-DOF point-mass equations."""

    nx: float
    nz: float
    phi_rad: float


class PointMass3DoFModel:
    """Nonlinear 3-DOF point-mass model with an acceleration input filter."""

    def __init__(self, config: PointMassModelConfig) -> None:
        self.config = config

    def acceleration_to_inputs(
        self,
        filtered_accel_mps2: Sequence[float],
        theta_rad: float,
        psi_rad: float,
    ) -> PointMassInputs:
        """Convert filtered ENU acceleration into ``N_x / N_z / phi``."""

        ax, ay, az = (
            self.clamp_acceleration(filtered_accel_mps2[0]),
            self.clamp_acceleration(filtered_accel_mps2[1]),
            self.clamp_acceleration(filtered_accel_mps2[2]),
        )
        force_east = ax
        force_north = ay
        force_up = az + self.config.gravity_mps2

        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_psi = math.cos(psi_rad)
        sin_psi = math.sin(psi_rad)

        a_v = (
            cos_theta * cos_psi * force_east
            + cos_theta * sin_psi * force_north
            + sin_theta * force_up
        )
        a_theta = (
            -sin_theta * cos_psi * force_east
            - sin_theta * sin_psi * force_north
            + cos_theta * force_up
        )
        a_psi = -sin_psi * force_east + cos_psi * force_north

        gravity = self.config.gravity_mps2
        nx = self._clamp(a_v / gravity, self.config.nx_min, self.config.nx_max)
        nz = self._clamp(
            math.hypot(a_theta, a_psi) / gravity,
            self.config.nz_min,
            self.config.nz_max,
        )
        phi_rad = self._clamp(
            math.atan2(a_psi, a_theta),
            self.config.phi_min_rad,
            self.config.phi_max_rad,
        )
        return PointMassInputs(nx, nz, phi_rad)

    def derivative(
        self,
        state: Sequence[float],
        control: Sequence[float],
        wind_velocity_mps: Sequence[float],
    ) -> list[float]:
        """Evaluate the augmented nonlinear state derivative."""

        (
            _x_m,
            _y_m,
            _altitude_m,
            speed_mps,
            theta_rad,
            psi_rad,
            ax_mps2,
            ay_mps2,
            az_mps2,
            ax_rate_mps3,
            ay_rate_mps3,
            az_rate_mps3,
        ) = state

        speed_for_denom = max(abs(speed_mps), self.config.min_speed_mps)
        cos_theta = math.cos(theta_rad)
        cos_theta_for_denom = self._safe_cos_theta(cos_theta)
        sin_theta = math.sin(theta_rad)

        inputs = self.acceleration_to_inputs(
            (ax_mps2, ay_mps2, az_mps2),
            theta_rad,
            psi_rad,
        )

        ground_vx = speed_mps * cos_theta * math.cos(psi_rad) + wind_velocity_mps[0]
        ground_vy = speed_mps * cos_theta * math.sin(psi_rad) + wind_velocity_mps[1]
        ground_vz = speed_mps * sin_theta + wind_velocity_mps[2]

        gravity = self.config.gravity_mps2
        speed_dot = gravity * (inputs.nx - sin_theta)
        theta_dot = (
            gravity
            / speed_for_denom
            * (inputs.nz * math.cos(inputs.phi_rad) - cos_theta)
        )
        psi_dot = (
            gravity
            * inputs.nz
            * math.sin(inputs.phi_rad)
            / (speed_for_denom * cos_theta_for_denom)
        )

        omega = self.config.natural_frequency_rad_s
        omega_squared = omega * omega
        damping = 2.0 * self.config.damping_ratio * omega
        cmd_ax, cmd_ay, cmd_az = (
            self.clamp_acceleration(control[0]),
            self.clamp_acceleration(control[1]),
            self.clamp_acceleration(control[2]),
        )

        return [
            ground_vx,
            ground_vy,
            ground_vz,
            speed_dot,
            theta_dot,
            psi_dot,
            ax_rate_mps3,
            ay_rate_mps3,
            az_rate_mps3,
            omega_squared * (cmd_ax - ax_mps2) - damping * ax_rate_mps3,
            omega_squared * (cmd_ay - ay_mps2) - damping * ay_rate_mps3,
            omega_squared * (cmd_az - az_mps2) - damping * az_rate_mps3,
        ]

    def step(
        self,
        state: Sequence[float],
        control: Sequence[float],
        wind_velocity_mps: Sequence[float],
        dt_s: float,
    ) -> list[float]:
        """Advance one zero-order-hold input interval using RK4."""

        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        k1 = self.derivative(state, control, wind_velocity_mps)
        k2 = self.derivative(self._add_scaled(state, k1, dt_s * 0.5), control, wind_velocity_mps)
        k3 = self.derivative(self._add_scaled(state, k2, dt_s * 0.5), control, wind_velocity_mps)
        k4 = self.derivative(self._add_scaled(state, k3, dt_s), control, wind_velocity_mps)
        stepped = [
            value + dt_s * (k1_value + 2.0 * k2_value + 2.0 * k3_value + k4_value) / 6.0
            for value, k1_value, k2_value, k3_value, k4_value in zip(state, k1, k2, k3, k4)
        ]
        return self._clamp_state_vector(stepped)

    def _clamp_state_vector(self, vector: list[float]) -> list[float]:
        vector[3] = max(self.config.min_speed_mps, vector[3])
        vector[4] = self._clamp(vector[4], -_MAX_ABS_THETA_RAD, _MAX_ABS_THETA_RAD)
        for index in (6, 7, 8):
            vector[index] = self.clamp_acceleration(vector[index])
        return vector

    def clamp_acceleration(self, value: float) -> float:
        limit = self.config.acceleration_command_limit_mps2
        return self._clamp(float(value), -limit, limit)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _safe_cos_theta(cos_theta: float) -> float:
        if abs(cos_theta) >= _COS_THETA_EPS:
            return cos_theta
        return _COS_THETA_EPS if cos_theta >= 0.0 else -_COS_THETA_EPS

    @staticmethod
    def _add_scaled(left: Sequence[float], right: Sequence[float], scale: float) -> list[float]:
        return [left_value + right_value * scale for left_value, right_value in zip(left, right)]


class ModelIterator:
    """Own and advance all UAV point-mass model instances."""

    DEFAULT_GRAVITY_MPS2 = 9.80665
    DEFAULT_MIN_SPEED_MPS = 1.0
    DEFAULT_NATURAL_FREQUENCY_RAD_S = 4.0
    DEFAULT_DAMPING_RATIO = 0.65
    DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2 = 6.0
    DEFAULT_NX_MIN = -1.0
    DEFAULT_NX_MAX = 1.0
    DEFAULT_NZ_MIN = 0.0
    DEFAULT_NZ_MAX = 4.0
    DEFAULT_PHI_MIN_DEG = -70.0
    DEFAULT_PHI_MAX_DEG = 70.0

    def __init__(self) -> None:
        self._config = self._default_config()
        self._system = PointMass3DoFModel(self._config)
        self._states: dict[str, AircraftState] = {}
        self._initial_states: dict[str, AircraftState] = {}
        self._baseline_health: dict[str, str] = {}
        self._cross_track_reference_y_m: dict[str, float] = {}
        self._controls: dict[str, AccelerationCommand] = {}
        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        self._time_s = 0.0

    def init(self, config: dict[str, object], seed: int) -> None:
        """Initialize all aircraft from the simulation configuration."""

        del seed
        self._config = self._parse_model_config(config.get("model", {}))
        self._system = PointMass3DoFModel(self._config)
        self._time_s = 0.0
        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        self._states = {}
        self._cross_track_reference_y_m = {}

        nodes = config.get("nodes", [])
        if nodes is None:
            nodes = []
        if not isinstance(nodes, list):
            raise ValueError("nodes must be a list")
        for index, node in enumerate(nodes):
            if not isinstance(node, dict):
                raise ValueError("each node must be an object")
            state = self._make_initial_state(node, index)
            if state.node_id in self._states:
                raise ValueError(f"duplicate node_id: {state.node_id}")
            self._states[state.node_id] = state
            self._cross_track_reference_y_m[state.node_id] = state.y_m - state.cross_track_error_m

        self._initial_states = {
            node_id: replace(state)
            for node_id, state in self._states.items()
        }
        self._baseline_health = {
            node_id: state.health
            for node_id, state in self._states.items()
        }
        self._controls = {
            node_id: AccelerationCommand()
            for node_id in self._states
        }

    def read_states(self) -> dict[str, AircraftState]:
        """Return detached state copies for algorithms and snapshots."""

        return {
            node_id: replace(state)
            for node_id, state in self._states.items()
        }

    def apply_controls(self, controls: Mapping[str, AccelerationCommand]) -> None:
        """Apply and saturate the latest ENU acceleration commands."""

        for node_id, control in controls.items():
            if node_id not in self._states:
                continue
            self._controls[node_id] = AccelerationCommand(
                self._system.clamp_acceleration(control.ax_cmd_mps2),
                self._system.clamp_acceleration(control.ay_cmd_mps2),
                self._system.clamp_acceleration(control.az_cmd_mps2),
            )

    def step(self, dt_s: float) -> None:
        """Advance all aircraft by one simulation step."""

        for node_id, state in self._states.items():
            previous_position = (state.x_m, state.y_m, state.altitude_m)
            control = self._controls.get(node_id, AccelerationCommand())
            vector = self._system.step(
                state.as_vector(),
                control.as_vector(),
                self._wind_velocity_mps,
                dt_s,
            )
            state.update_from_vector(vector)
            self._update_derived_state(state, previous_position)
        self._time_s += dt_s

    def tick(self, dt_s: float) -> None:
        """Compatibility alias for the HLD tick interface."""

        self.step(dt_s)

    def inject_wind(self, command: object) -> None:
        """Set a constant wind velocity from a disturbance command."""

        params = self._command_params(command)
        speed_mps = float(params.get("speed_mps", 0.0))
        direction_rad = math.radians(float(params.get("direction_deg", 0.0)))
        vertical_mps = float(params.get("vertical_mps", 0.0))
        self._wind_velocity_mps = (
            speed_mps * math.cos(direction_rad),
            speed_mps * math.sin(direction_rad),
            vertical_mps,
        )

    def inject_fault(self, command: object) -> None:
        """Update node health; control degradation is handled by the algorithm."""

        target = self._command_value(command, "target")
        if target is None or str(target) not in self._states:
            return
        params = self._command_params(command)
        self._states[str(target)].health = str(params.get("mode", "degraded"))

    def clear_faults(self) -> None:
        """Clear dynamic model disturbances and restore baseline health."""

        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        for node_id, state in self._states.items():
            state.health = self._baseline_health.get(node_id, "normal")

    def reset(self) -> None:
        """Restore configured initial states without rereading configuration."""

        self._time_s = 0.0
        self._wind_velocity_mps = (0.0, 0.0, 0.0)
        self._states = {
            node_id: replace(state)
            for node_id, state in self._initial_states.items()
        }
        self._controls = {
            node_id: AccelerationCommand()
            for node_id in self._states
        }

    def close(self) -> None:
        self._states.clear()
        self._initial_states.clear()
        self._baseline_health.clear()
        self._cross_track_reference_y_m.clear()
        self._controls.clear()

    def _make_initial_state(self, node: dict[str, object], index: int) -> AircraftState:
        node_id = str(node.get("node_id") or node.get("id") or f"A{index + 1:02d}")
        role = str(node.get("role") or ("leader" if index == 0 else "wingman"))
        speed_mps = max(self._config.min_speed_mps, float(node.get("speed_mps", 5.0)))

        theta_rad = math.radians(float(node.get("theta_deg", 0.0)))
        psi_rad = math.radians(float(node.get("psi_v_deg", 0.0)))
        has_velocity_components = any(
            key in node
            for key in ("vx_mps", "vy_mps", "vz_mps", "climb_rate_mps")
        )
        if has_velocity_components:
            default_vx = speed_mps * math.cos(theta_rad) * math.cos(psi_rad)
            default_vy = speed_mps * math.cos(theta_rad) * math.sin(psi_rad)
            default_vz = speed_mps * math.sin(theta_rad)
            vx_mps = float(node.get("vx_mps", default_vx))
            vy_mps = float(node.get("vy_mps", default_vy))
            vz_mps = float(node.get("vz_mps", node.get("climb_rate_mps", default_vz)))
            speed_mps = max(self._config.min_speed_mps, math.sqrt(vx_mps * vx_mps + vy_mps * vy_mps + vz_mps * vz_mps))
            if "theta_deg" not in node:
                theta_rad = math.asin(max(-1.0, min(1.0, vz_mps / speed_mps)))
            if "psi_v_deg" not in node and math.hypot(vx_mps, vy_mps) > 1e-9:
                psi_rad = math.atan2(vy_mps, vx_mps)

        if any(key in node for key in ("ax_mps2", "ay_mps2", "az_mps2")):
            ax_mps2 = float(node.get("ax_mps2", 0.0))
            ay_mps2 = float(node.get("ay_mps2", 0.0))
            az_mps2 = float(node.get("az_mps2", 0.0))
        else:
            nx = float(node.get("nx", math.sin(theta_rad)))
            nz = float(node.get("nz", math.cos(theta_rad)))
            phi_rad = math.radians(float(node.get("phi_deg", 0.0)))
            ax_mps2, ay_mps2, az_mps2 = self._inputs_to_enu_acceleration(
                nx,
                nz,
                phi_rad,
                theta_rad,
                psi_rad,
            )

        state = AircraftState(
            node_id=node_id,
            role=role,
            health=str(node.get("health", "normal")),
            x_m=float(node.get("x_m", index * -45.0)),
            y_m=float(node.get("y_m", 0.0 if index == 0 else (index * 2 - 3) * 50.0)),
            altitude_m=float(node.get("altitude_m", 1200.0 + index * 15.0)),
            speed_mps=speed_mps,
            theta_rad=theta_rad,
            psi_rad=psi_rad,
            ax_mps2=ax_mps2,
            ay_mps2=ay_mps2,
            az_mps2=az_mps2,
            ax_rate_mps3=float(node.get("ax_rate_mps3", 0.0)),
            ay_rate_mps3=float(node.get("ay_rate_mps3", 0.0)),
            az_rate_mps3=float(node.get("az_rate_mps3", 0.0)),
            nx=0.0,
            nz=0.0,
            phi_rad=0.0,
            cross_track_error_m=float(node.get("cross_track_error_m", 0.0)),
            distance_to_go_m=float(node.get("distance_to_go_m", 6000.0)),
        )
        self._update_inputs_from_filtered_acceleration(state)
        return state

    def _update_derived_state(
        self,
        state: AircraftState,
        previous_position: tuple[float, float, float],
    ) -> None:
        self._update_inputs_from_filtered_acceleration(state)
        state.cross_track_error_m = (
            state.y_m - self._cross_track_reference_y_m[state.node_id]
        )
        travelled_m = math.dist(previous_position, (state.x_m, state.y_m, state.altitude_m))
        state.distance_to_go_m = max(0.0, state.distance_to_go_m - travelled_m)

    def _update_inputs_from_filtered_acceleration(self, state: AircraftState) -> None:
        inputs = self._system.acceleration_to_inputs(
            (state.ax_mps2, state.ay_mps2, state.az_mps2),
            state.theta_rad,
            state.psi_rad,
        )
        state.nx = inputs.nx
        state.nz = inputs.nz
        state.phi_rad = inputs.phi_rad

    def _inputs_to_enu_acceleration(
        self,
        nx: float,
        nz: float,
        phi_rad: float,
        theta_rad: float,
        psi_rad: float,
    ) -> tuple[float, float, float]:
        gravity = self._config.gravity_mps2
        a_v = gravity * nx
        a_theta = gravity * nz * math.cos(phi_rad)
        a_psi = gravity * nz * math.sin(phi_rad)

        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        cos_psi = math.cos(psi_rad)
        sin_psi = math.sin(psi_rad)

        force_east = (
            cos_theta * cos_psi * a_v
            - sin_theta * cos_psi * a_theta
            - sin_psi * a_psi
        )
        force_north = (
            cos_theta * sin_psi * a_v
            - sin_theta * sin_psi * a_theta
            + cos_psi * a_psi
        )
        force_up = sin_theta * a_v + cos_theta * a_theta
        return (force_east, force_north, force_up - gravity)

    @classmethod
    def _default_config(cls) -> PointMassModelConfig:
        return PointMassModelConfig(
            gravity_mps2=cls.DEFAULT_GRAVITY_MPS2,
            min_speed_mps=cls.DEFAULT_MIN_SPEED_MPS,
            natural_frequency_rad_s=cls.DEFAULT_NATURAL_FREQUENCY_RAD_S,
            damping_ratio=cls.DEFAULT_DAMPING_RATIO,
            acceleration_command_limit_mps2=cls.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2,
            nx_min=cls.DEFAULT_NX_MIN,
            nx_max=cls.DEFAULT_NX_MAX,
            nz_min=cls.DEFAULT_NZ_MIN,
            nz_max=cls.DEFAULT_NZ_MAX,
            phi_min_rad=math.radians(cls.DEFAULT_PHI_MIN_DEG),
            phi_max_rad=math.radians(cls.DEFAULT_PHI_MAX_DEG),
        )

    @classmethod
    def _parse_model_config(cls, raw_config: object) -> PointMassModelConfig:
        if raw_config is None:
            raw_config = {}
        if not isinstance(raw_config, Mapping):
            raise ValueError("model must be an object")

        filter_config = raw_config.get("acceleration_filter", {})
        if filter_config is None:
            filter_config = {}
        if not isinstance(filter_config, Mapping):
            raise ValueError("model.acceleration_filter must be an object")

        limits_config = raw_config.get("limits", {})
        if limits_config is None:
            limits_config = {}
        if not isinstance(limits_config, Mapping):
            raise ValueError("model.limits must be an object")

        natural_frequency = float(
            filter_config.get(
                "natural_frequency_rad_s",
                raw_config.get("natural_frequency_rad_s", cls.DEFAULT_NATURAL_FREQUENCY_RAD_S),
            )
        )
        damping_ratio = float(
            filter_config.get(
                "damping_ratio",
                raw_config.get("damping_ratio", cls.DEFAULT_DAMPING_RATIO),
            )
        )
        acceleration_limit = float(
            limits_config.get(
                "acceleration_command_mps2",
                raw_config.get("max_acceleration_mps2", cls.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2),
            )
        )
        gravity = float(raw_config.get("gravity_mps2", cls.DEFAULT_GRAVITY_MPS2))
        min_speed = float(raw_config.get("min_speed_mps", cls.DEFAULT_MIN_SPEED_MPS))

        config = PointMassModelConfig(
            gravity_mps2=gravity,
            min_speed_mps=min_speed,
            natural_frequency_rad_s=natural_frequency,
            damping_ratio=damping_ratio,
            acceleration_command_limit_mps2=acceleration_limit,
            nx_min=float(limits_config.get("nx_min", cls.DEFAULT_NX_MIN)),
            nx_max=float(limits_config.get("nx_max", cls.DEFAULT_NX_MAX)),
            nz_min=float(limits_config.get("nz_min", cls.DEFAULT_NZ_MIN)),
            nz_max=float(limits_config.get("nz_max", cls.DEFAULT_NZ_MAX)),
            phi_min_rad=math.radians(float(limits_config.get("phi_min_deg", cls.DEFAULT_PHI_MIN_DEG))),
            phi_max_rad=math.radians(float(limits_config.get("phi_max_deg", cls.DEFAULT_PHI_MAX_DEG))),
        )
        cls._validate_config(config)
        return config

    @staticmethod
    def _validate_config(config: PointMassModelConfig) -> None:
        if config.gravity_mps2 <= 0.0:
            raise ValueError("model.gravity_mps2 must be positive")
        if config.min_speed_mps <= 0.0:
            raise ValueError("model.min_speed_mps must be positive")
        if config.natural_frequency_rad_s <= 0.0:
            raise ValueError("model.acceleration_filter.natural_frequency_rad_s must be positive")
        if not 0.0 < config.damping_ratio < 1.0:
            raise ValueError("model.acceleration_filter.damping_ratio must be in (0, 1)")
        if config.acceleration_command_limit_mps2 <= 0.0:
            raise ValueError("model.limits.acceleration_command_mps2 must be positive")
        if config.nx_min >= config.nx_max:
            raise ValueError("model.limits.nx_min must be less than nx_max")
        if config.nz_min < 0.0 or config.nz_min >= config.nz_max:
            raise ValueError("model.limits.nz_min must be non-negative and less than nz_max")
        if config.phi_min_rad >= config.phi_max_rad:
            raise ValueError("model.limits.phi_min_deg must be less than phi_max_deg")

    @staticmethod
    def _command_params(command: object) -> Mapping[str, object]:
        params = ModelIterator._command_value(command, "params")
        return params if isinstance(params, Mapping) else {}

    @staticmethod
    def _command_value(command: object, name: str) -> object:
        if isinstance(command, Mapping):
            return command.get(name)
        return getattr(command, name, None)
