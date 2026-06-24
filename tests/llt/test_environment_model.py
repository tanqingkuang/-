"""Unit tests for the 3-DOF point-mass model (src/environment/model.py)."""

from __future__ import annotations

import math
import unittest

from src.environment.model import (
    AccelerationCommand,
    ModelIterator,
    PointMass3DoFModel,
    PointMassModelConfig,
    STATE_SIZE,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_config() -> PointMassModelConfig:
    return ModelIterator._default_config()


def _model() -> PointMass3DoFModel:
    return PointMass3DoFModel(_default_config())


def _level_trim_state(speed_mps: float = 10.0) -> list[float]:
    """Straight-and-level initial state: E=N=U=0, heading north, zero accel rates."""
    psi_rad = math.radians(90.0)  # north
    return [0.0, 0.0, 1200.0, speed_mps, 0.0, psi_rad, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Config validation
# ---------------------------------------------------------------------------

class TestConfigValidation(unittest.TestCase):
    def test_gravity_must_be_positive(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "gravity"):
            ModelIterator._validate_config(
                PointMassModelConfig(
                    **{**cfg.__dict__, "gravity_mps2": 0.0}
                )
            )

    def test_min_speed_must_be_positive(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "min_speed"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "min_speed_mps": -1.0})
            )

    def test_natural_frequency_must_be_positive(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "natural_frequency"):
            ModelIterator._validate_config(
                PointMassModelConfig(
                    **{**cfg.__dict__, "natural_frequency_rad_s": 0.0}
                )
            )

    def test_damping_ratio_must_be_strictly_between_0_and_1(self):
        cfg = _default_config()
        for bad in (0.0, 1.0, -0.1, 1.1):
            with self.assertRaisesRegex(ValueError, "damping_ratio"):
                ModelIterator._validate_config(
                    PointMassModelConfig(**{**cfg.__dict__, "damping_ratio": bad})
                )

    def test_nx_min_must_be_less_than_nx_max(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "nx_min"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "nx_min": 1.0, "nx_max": 1.0})
            )

    def test_nz_min_must_be_non_negative(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "nz_min"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "nz_min": -0.1})
            )

    def test_phi_min_must_be_less_than_phi_max(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "phi_min"):
            ModelIterator._validate_config(
                PointMassModelConfig(
                    **{**cfg.__dict__,
                       "phi_min_rad": math.radians(10.0),
                       "phi_max_rad": math.radians(10.0)}
                )
            )

    def test_nz_max_must_cover_trim(self):
        """nz_max < 1.0 makes level-flight trim unreachable — must raise."""
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "nz_max"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "nz_max": 0.9})
            )

    def test_nz_min_must_not_exceed_trim(self):
        """nz_min > 1.0 excludes level-flight trim nz=1 from the range — must raise."""
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "nz"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "nz_min": 1.5, "nz_max": 4.0})
            )

    def test_nx_range_must_include_zero(self):
        """nx_min > 0 means zero thrust is unreachable — must raise."""
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "nx"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "nx_min": 0.1, "nx_max": 1.0})
            )

    def test_phi_range_must_include_zero(self):
        """phi_min > 0 means straight flight is unreachable — must raise."""
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "phi"):
            ModelIterator._validate_config(
                PointMassModelConfig(
                    **{**cfg.__dict__,
                       "phi_min_rad": math.radians(10.0),
                       "phi_max_rad": math.radians(70.0)}
                )
            )

    def test_parse_model_config_raises_on_invalid_type(self):
        with self.assertRaisesRegex(ValueError, "model must be an object"):
            ModelIterator._parse_model_config("not_a_dict")

    def test_parse_model_config_defaults_are_valid(self):
        cfg = ModelIterator._parse_model_config({})
        self.assertEqual(cfg.gravity_mps2, ModelIterator.DEFAULT_GRAVITY_MPS2)
        self.assertEqual(cfg.damping_ratio, ModelIterator.DEFAULT_DAMPING_RATIO)
        self.assertEqual(cfg.max_climb_rate_mps, ModelIterator.DEFAULT_MAX_CLIMB_RATE_MPS)
        self.assertEqual(cfg.max_descent_rate_mps, ModelIterator.DEFAULT_MAX_DESCENT_RATE_MPS)

    def test_vertical_rate_limits_must_be_positive(self):
        cfg = _default_config()
        with self.assertRaisesRegex(ValueError, "max_climb_rate"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "max_climb_rate_mps": 0.0})
            )
        with self.assertRaisesRegex(ValueError, "max_descent_rate"):
            ModelIterator._validate_config(
                PointMassModelConfig(**{**cfg.__dict__, "max_descent_rate_mps": 0.0})
            )


# ---------------------------------------------------------------------------
# Round-trip: ENU acceleration ↔ Nx / Nz / phi
# ---------------------------------------------------------------------------

class TestAccelerationRoundTrip(unittest.TestCase):
    def test_level_trim_converts_to_nx0_nz1_phi0(self):
        """Zero ENU command at level flight → Nx=0, Nz=1, phi=0."""
        m = _model()
        inputs = m.acceleration_to_inputs([0.0, 0.0, 0.0], 0.0, 0.0)
        self.assertAlmostEqual(inputs.nx, 0.0, delta=1e-9)
        self.assertAlmostEqual(inputs.nz, 1.0, delta=1e-9)
        self.assertAlmostEqual(inputs.phi_rad, 0.0, delta=1e-9)

    def test_positive_east_command_heading_east(self):
        """Pure east acceleration, heading east → positive Nx (forward thrust)."""
        m = _model()
        inputs = m.acceleration_to_inputs([3.0, 0.0, 0.0], 0.0, 0.0)
        self.assertGreater(inputs.nx, 0.0)

    def test_positive_north_command_heading_east_turns_left(self):
        """North cmd while heading east → CCW turn (psi increasing), phi > 0 in this model's convention."""
        m = _model()
        inputs = m.acceleration_to_inputs([0.0, 2.0, 0.0], 0.0, 0.0)
        # psi_dot = g * nz * sin(phi) / speed; to turn north (CCW), psi_dot > 0 → phi > 0
        self.assertGreater(inputs.phi_rad, 0.0)

    def test_positive_up_command_increases_nz(self):
        """Up command adds lift force → Nz > 1."""
        m = _model()
        inputs = m.acceleration_to_inputs([0.0, 0.0, 3.0], 0.0, 0.0)
        self.assertGreater(inputs.nz, 1.0)

    def test_inverse_map_is_consistent(self):
        """iterator._inputs_to_enu then model.acceleration_to_inputs should return original inputs."""
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 10.0}]}, seed=0)

        theta = math.radians(5.0)
        psi = math.radians(45.0)
        nx_in, nz_in, phi_in = 0.1, 1.05, math.radians(15.0)

        ax, ay, az = it._inputs_to_enu_acceleration(nx_in, nz_in, phi_in, theta, psi)
        result = it._system.acceleration_to_inputs([ax, ay, az], theta, psi)

        self.assertAlmostEqual(result.nx, nx_in, delta=1e-6)
        self.assertAlmostEqual(result.nz, nz_in, delta=1e-6)
        self.assertAlmostEqual(result.phi_rad, phi_in, delta=1e-6)


# ---------------------------------------------------------------------------
# Saturation / clamping
# ---------------------------------------------------------------------------

class TestSaturation(unittest.TestCase):
    def test_clamp_acceleration_limits_output(self):
        m = _model()
        limit = m.config.acceleration_command_limit_mps2
        self.assertAlmostEqual(m.clamp_acceleration(limit + 10.0), limit)
        self.assertAlmostEqual(m.clamp_acceleration(-(limit + 10.0)), -limit)
        self.assertAlmostEqual(m.clamp_acceleration(0.5), 0.5)

    def test_state_vector_speed_clamped_to_min(self):
        m = _model()
        state = _level_trim_state(speed_mps=10.0)
        state[3] = -5.0
        stepped = m.step(state, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt_s=0.01)
        self.assertGreaterEqual(stepped[3], m.config.min_speed_mps)

    def test_nx_clamped_by_config_limits(self):
        m = _model()
        inputs = m.acceleration_to_inputs([100.0, 0.0, 0.0], 0.0, 0.0)
        self.assertLessEqual(inputs.nx, m.config.nx_max)

    def test_nz_clamped_by_config_limits(self):
        m = _model()
        inputs = m.acceleration_to_inputs([0.0, 0.0, 100.0], 0.0, 0.0)
        self.assertLessEqual(inputs.nz, m.config.nz_max)

    def test_climb_rate_is_limited_by_config(self):
        cfg = PointMassModelConfig(
            **{**_default_config().__dict__, "max_climb_rate_mps": 2.0}
        )
        m = PointMass3DoFModel(cfg)
        state = _level_trim_state(speed_mps=10.0)
        state[4] = math.radians(45.0)

        stepped = m.step(state, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt_s=0.01)

        self.assertLessEqual(stepped[3] * math.sin(stepped[4]), 2.0 + 1e-9)

    def test_descent_rate_is_limited_by_config(self):
        cfg = PointMassModelConfig(
            **{**_default_config().__dict__, "max_descent_rate_mps": 3.0}
        )
        m = PointMass3DoFModel(cfg)
        state = _level_trim_state(speed_mps=10.0)
        state[4] = math.radians(-45.0)

        stepped = m.step(state, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt_s=0.01)

        self.assertGreaterEqual(stepped[3] * math.sin(stepped[4]), -3.0 - 1e-9)


# ---------------------------------------------------------------------------
# Level trim: straight-and-level doesn't drift
# ---------------------------------------------------------------------------

class TestLevelTrim(unittest.TestCase):
    def test_level_trim_no_drift(self):
        """With zero ENU command and level trim accel state, altitude should stay constant."""
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 15.0, "psi_v_deg": 0.0}]}, seed=0)
        it.apply_controls({"A": AccelerationCommand(0.0, 0.0, 0.0)})

        initial_altitude = it.read_states()["A"].altitude_m
        for _ in range(200):
            it.step(0.01)

        final_altitude = it.read_states()["A"].altitude_m
        self.assertAlmostEqual(final_altitude, initial_altitude, delta=0.5)

    def test_level_trim_speed_stable(self):
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 15.0}]}, seed=0)
        it.apply_controls({"A": AccelerationCommand(0.0, 0.0, 0.0)})
        initial_speed = it.read_states()["A"].speed_mps
        for _ in range(200):
            it.step(0.01)
        final_speed = it.read_states()["A"].speed_mps
        self.assertAlmostEqual(final_speed, initial_speed, delta=0.5)


# ---------------------------------------------------------------------------
# Second-order filter response (underdamped, ω=4, ζ=0.65)
# ---------------------------------------------------------------------------

class TestFilterResponse(unittest.TestCase):
    def test_step_response_reaches_command(self):
        """Sustained az command should bring filtered az close to the command value."""
        m = _model()
        cmd_az = 2.0
        state = _level_trim_state()
        for _ in range(500):
            state = m.step(state, [0.0, 0.0, cmd_az], [0.0, 0.0, 0.0], dt_s=0.01)
        self.assertLess(abs(state[8] - cmd_az) / cmd_az, 0.05)

    def test_underdamped_overshoot_occurs(self):
        """ζ=0.65 → about 8 % overshoot; peak must exceed the command."""
        m = _model()
        cmd_az = 2.0
        state = _level_trim_state()
        peak = 0.0
        for _ in range(300):
            state = m.step(state, [0.0, 0.0, cmd_az], [0.0, 0.0, 0.0], dt_s=0.01)
            peak = max(peak, state[8])
        self.assertGreater(peak, cmd_az * 1.01)


# ---------------------------------------------------------------------------
# Wind disturbance
# ---------------------------------------------------------------------------

class TestWindDisturbance(unittest.TestCase):
    def test_wind_moves_ground_position(self):
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 15.0}]}, seed=0)
        it.apply_controls({"A": AccelerationCommand(0.0, 0.0, 0.0)})
        it.inject_wind({"params": {"speed_mps": 5.0, "direction_deg": 0.0}})

        initial_x = it.read_states()["A"].x_m
        for _ in range(100):
            it.step(0.01)
        final_x = it.read_states()["A"].x_m
        self.assertGreater(final_x, initial_x)

    def test_clear_wind_removes_wind(self):
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 15.0}]}, seed=0)
        it.inject_wind({"params": {"speed_mps": 10.0, "direction_deg": 0.0}})
        it.clear_wind()
        self.assertEqual(it._wind_velocity_mps, (0.0, 0.0, 0.0))



# ---------------------------------------------------------------------------
# Reset behaviour
# ---------------------------------------------------------------------------

class TestReset(unittest.TestCase):
    def test_reset_restores_initial_state(self):
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A01", "x_m": 0.0, "speed_mps": 10.0}]}, seed=0)
        initial_x = it.read_states()["A01"].x_m
        initial_speed = it.read_states()["A01"].speed_mps

        it.apply_controls({"A01": AccelerationCommand(3.0, 0.0, 0.0)})
        for _ in range(100):
            it.step(0.01)

        it.reset()
        after = it.read_states()["A01"]
        self.assertAlmostEqual(after.x_m, initial_x, delta=1e-9)
        self.assertAlmostEqual(after.speed_mps, initial_speed, delta=1e-9)

    def test_reset_clears_wind(self):
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 10.0}]}, seed=0)
        it.inject_wind({"params": {"speed_mps": 5.0, "direction_deg": 0.0}})
        it.reset()
        self.assertEqual(it._wind_velocity_mps, (0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# RK4 single step sanity
# ---------------------------------------------------------------------------

class TestRK4Step(unittest.TestCase):
    def test_step_requires_positive_dt(self):
        m = _model()
        with self.assertRaisesRegex(ValueError, "dt_s must be positive"):
            m.step(_level_trim_state(), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt_s=0.0)

    def test_step_returns_correct_size(self):
        m = _model()
        result = m.step(_level_trim_state(), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt_s=0.01)
        self.assertEqual(len(result), STATE_SIZE)

    def test_step_position_advances_with_speed(self):
        """Heading east at 10 m/s → east position increases."""
        m = _model()
        state = _level_trim_state(speed_mps=10.0)
        state[5] = 0.0  # psi = 0 (east)
        initial_x = state[0]
        stepped = m.step(state, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt_s=0.1)
        self.assertGreater(stepped[0], initial_x)


# ---------------------------------------------------------------------------
# _make_initial_state: velocity-component initialisation path
# ---------------------------------------------------------------------------

class TestInitialStateVelocityComponents(unittest.TestCase):
    def test_vx_vy_override_derives_psi(self):
        """Providing vx/vy should override psi_v_deg derived from default 0."""
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 10.0, "vx_mps": 0.0, "vy_mps": 10.0}]}, seed=0)
        state = it.read_states()["A"]
        self.assertAlmostEqual(state.psi_rad, math.radians(90.0), delta=1e-6)

    def test_vz_derives_theta(self):
        """Positive vz while not specifying theta_deg → positive climb angle."""
        speed = 10.0
        vz = 2.0
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": speed, "vx_mps": 0.0, "vy_mps": speed, "vz_mps": vz}]}, seed=0)
        state = it.read_states()["A"]
        expected_theta = math.asin(vz / math.sqrt(speed ** 2 + vz ** 2))
        self.assertAlmostEqual(state.theta_rad, expected_theta, delta=1e-6)

    def test_explicit_theta_deg_not_overridden_by_vz(self):
        """If theta_deg is supplied alongside velocity components, it takes priority."""
        it = ModelIterator()
        it.init({"nodes": [{
            "node_id": "A", "speed_mps": 10.0,
            "vx_mps": 0.0, "vy_mps": 10.0, "vz_mps": 5.0,
            "theta_deg": 0.0,
        }]}, seed=0)
        state = it.read_states()["A"]
        self.assertAlmostEqual(state.theta_rad, 0.0, delta=1e-9)

    def test_initial_vz_is_clamped_by_vertical_rate_limit(self):
        """初始垂向速度分量超过包线时，应裁剪航迹倾角而不是保留不可飞爬升率。"""
        it = ModelIterator()
        it.init(
            {
                "model": {"limits": {"max_climb_rate_mps": 2.0}},
                "nodes": [{"node_id": "A", "speed_mps": 10.0, "vx_mps": 10.0, "vy_mps": 0.0, "vz_mps": 6.0}],
            },
            seed=0,
        )

        state = it.read_states()["A"]

        self.assertLessEqual(state.vz_mps, 2.0 + 1e-9)


# ---------------------------------------------------------------------------
# _parse_model_config: legacy field compatibility
# ---------------------------------------------------------------------------

class TestLegacyConfigFields(unittest.TestCase):
    def test_flat_natural_frequency_accepted(self):
        """Old-style flat natural_frequency_rad_s should map correctly."""
        cfg = ModelIterator._parse_model_config({"natural_frequency_rad_s": 3.0})
        self.assertAlmostEqual(cfg.natural_frequency_rad_s, 3.0)

    def test_flat_damping_ratio_accepted(self):
        cfg = ModelIterator._parse_model_config({"damping_ratio": 0.7})
        self.assertAlmostEqual(cfg.damping_ratio, 0.7)

    def test_flat_max_acceleration_accepted(self):
        """Old-style max_acceleration_mps2 should map to acceleration_command_limit."""
        cfg = ModelIterator._parse_model_config({"max_acceleration_mps2": 8.0})
        self.assertAlmostEqual(cfg.acceleration_command_limit_mps2, 8.0)

    def test_nested_overrides_flat(self):
        """acceleration_filter.natural_frequency_rad_s takes priority over flat key."""
        cfg = ModelIterator._parse_model_config({
            "natural_frequency_rad_s": 2.0,
            "acceleration_filter": {"natural_frequency_rad_s": 5.0},
        })
        self.assertAlmostEqual(cfg.natural_frequency_rad_s, 5.0)

    def test_vertical_rate_limits_parse_from_nested_limits(self):
        cfg = ModelIterator._parse_model_config({
            "limits": {"max_climb_rate_mps": 4.0, "max_descent_rate_mps": 5.0},
        })

        self.assertAlmostEqual(cfg.max_climb_rate_mps, 4.0)
        self.assertAlmostEqual(cfg.max_descent_rate_mps, 5.0)


# ---------------------------------------------------------------------------
# psi_rad normalization
# ---------------------------------------------------------------------------

class TestPsiNormalization(unittest.TestCase):
    def test_compute_psi_dot_rad_s_fixed_values(self):
        """固定数值验证公式幅值和单位：g=10, nz=2, phi=30°, V=20, cos_theta=0.5 → 精确 1 rad/s。"""
        # 手算：10 × 2 × sin(30°) / (20 × 0.5) = 10 × 2 × 0.5 / 10 = 1.0 rad/s
        result = PointMass3DoFModel.compute_psi_dot_rad_s(
            gravity=10.0,
            nz=2.0,
            phi_rad=math.radians(30.0),
            speed=20.0,
            cos_theta=0.5,
        )
        self.assertAlmostEqual(result, 1.0, delta=1e-12)

    def test_psi_dot_deg_s_level_flight_is_zero(self):
        """水平飞行零滚转时偏航角速率应为零。"""
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 50.0}]}, seed=0)
        state = it.read_states()["A"]
        self.assertAlmostEqual(state.psi_dot_deg_s, 0.0, delta=1e-6)

    def test_psi_dot_deg_s_positive_roll_positive_turn(self):
        """正滚转角（左倾）应产生正偏航角速率（左转/逆时针）。"""
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 50.0}]}, seed=0)
        limit = ModelIterator.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2
        it.apply_controls({"A": AccelerationCommand(0.0, limit, 0.0)})
        for _ in range(20):
            it.step(0.005)
        state = it.read_states()["A"]
        self.assertGreater(state.phi_rad, 0.0)
        self.assertGreater(state.psi_dot_deg_s, 0.0)

    def test_psi_dot_deg_s_uses_config_min_speed(self):
        """偏航角速率应使用配置最低速度而非硬编码 1.0；初速等于 min_speed 以触发分母保护分支。"""
        min_speed = 0.5
        it = ModelIterator()
        # 初速设为 min_speed，积分后速度被夹在 0.5；旧硬编码 1.0 会使分母翻倍导致结果差 2 倍。
        it.init({"nodes": [{"node_id": "A", "speed_mps": min_speed}],
                 "model": {"min_speed_mps": min_speed}}, seed=0)
        limit = ModelIterator.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2
        it.apply_controls({"A": AccelerationCommand(0.0, limit, 0.0)})
        for _ in range(20):
            it.step(0.005)
        state = it.read_states()["A"]
        import math as _math
        cos_theta = PointMass3DoFModel._safe_cos_theta(_math.cos(state.theta_rad))
        speed = max(min_speed, state.speed_mps)
        expected = _math.degrees(
            PointMass3DoFModel.compute_psi_dot_rad_s(
                it._config.gravity_mps2, state.nz, state.phi_rad, speed, cos_theta
            )
        )
        self.assertAlmostEqual(state.psi_dot_deg_s, expected, delta=1e-6)
        # 旧硬编码 1.0 时结果为 expected/2，确保差异大到足以被检出。
        self.assertGreater(abs(state.psi_dot_deg_s), 1.0)

    def test_psi_dot_deg_s_uses_config_gravity(self):
        """偏航角速率应使用配置重力而非硬编码值。"""
        gravity = 4.0
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 50.0}],
                 "model": {"gravity_mps2": gravity}}, seed=0)
        limit = ModelIterator.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2
        it.apply_controls({"A": AccelerationCommand(0.0, limit, 0.0)})
        for _ in range(20):
            it.step(0.005)
        state = it.read_states()["A"]
        import math as _math
        cos_theta = PointMass3DoFModel._safe_cos_theta(_math.cos(state.theta_rad))
        speed = max(it._config.min_speed_mps, state.speed_mps)
        expected = _math.degrees(
            PointMass3DoFModel.compute_psi_dot_rad_s(
                gravity, state.nz, state.phi_rad, speed, cos_theta
            )
        )
        self.assertAlmostEqual(state.psi_dot_deg_s, expected, delta=1e-6)

    def test_psi_stays_within_pi_after_full_circle(self):
        """After a sustained hard turn, psi_rad must remain in (-pi, pi]."""
        it = ModelIterator()
        it.init({"nodes": [{"node_id": "A", "speed_mps": 50.0}]}, seed=0)
        limit = ModelIterator.DEFAULT_ACCELERATION_COMMAND_LIMIT_MPS2
        it.apply_controls({"A": AccelerationCommand(0.0, limit, 0.0)})
        for _ in range(600):  # 3 s at dt=0.005 — more than one full revolution
            it.step(0.005)
        state = it.read_states()["A"]
        self.assertGreater(state.psi_rad, -math.pi)
        self.assertLessEqual(state.psi_rad, math.pi)


if __name__ == "__main__":
    unittest.main()
