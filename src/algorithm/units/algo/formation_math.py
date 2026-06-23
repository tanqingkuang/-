"""Stateless math helpers for formation algorithms."""

from __future__ import annotations

import math

from src.algorithm.context.leaf_types import MotionProfS


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp ``value`` to the closed interval [lower, upper]."""

    if lower > upper:
        raise ValueError("lower must be <= upper")
    return max(lower, min(upper, value))


def enu_to_track(vector: tuple[float, float, float], state: MotionProfS) -> tuple[float, float, float]:
    """Transform an ENU vector to local track axes: forward, lateral, vertical."""

    forward, lateral, vertical = _track_basis(state)
    return (
        _dot(vector, forward),
        _dot(vector, lateral),
        _dot(vector, vertical),
    )


def track_to_enu(vector: tuple[float, float, float], state: MotionProfS) -> tuple[float, float, float]:
    """Transform a local track vector to ENU."""

    forward, lateral, vertical = _track_basis(state)
    return (
        vector[0] * forward[0] + vector[1] * lateral[0] + vector[2] * vertical[0],
        vector[0] * forward[1] + vector[1] * lateral[1] + vector[2] * vertical[1],
        vector[0] * forward[2] + vector[1] * lateral[2] + vector[2] * vertical[2],
    )


def horizontal_track_basis(state: MotionProfS) -> tuple[float, float]:
    """Return the leader horizontal track unit vector in ENU."""

    vx = state.vd.vEast
    vy = state.vd.vNorth
    ground = math.hypot(vx, vy)
    if ground <= 0.0:
        raise ValueError("horizontal track frame requires non-zero horizontal velocity")
    return vx / ground, vy / ground


def horizontal_track_to_enu(vector: tuple[float, float], state: MotionProfS) -> tuple[float, float]:
    """Transform a horizontal track vector to ENU without coupling vertical velocity."""

    return horizontal_track_vector_to_enu(vector, horizontal_track_basis(state))


def horizontal_track_vector_to_enu(vector: tuple[float, float], track: tuple[float, float]) -> tuple[float, float]:
    """Transform a horizontal track vector to ENU using a precomputed track basis."""

    track_x, track_y = track
    return (
        vector[0] * track_x - vector[1] * track_y,
        vector[0] * track_y + vector[1] * track_x,
    )


def _track_basis(state: MotionProfS) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    vx = state.vd.vEast
    vy = state.vd.vNorth
    vz = state.vd.vUp
    ground = math.hypot(vx, vy)
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    if speed <= 0.0 or ground <= 0.0:
        raise ValueError("track frame requires non-zero horizontal velocity")

    cos_theta = ground / speed
    sin_theta = vz / speed
    cos_psi = vx / ground
    sin_psi = vy / ground
    forward = (cos_theta * cos_psi, cos_theta * sin_psi, sin_theta)
    lateral = (-sin_psi, cos_psi, 0.0)
    vertical = (-sin_theta * cos_psi, -sin_theta * sin_psi, cos_theta)
    return forward, lateral, vertical


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
