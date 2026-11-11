#!/usr/bin/env python3
"""Pure helpers for the SO-101 leader-to-Isaac UDP protocol.

The value mapping below is the one NVIDIA's Sim-to-Real SO-101 Workshop uses
to drive the SO-ARM101 USD articulation from a lerobot SO-101 leader
(`source/sim_to_real_so101/utils/lerobot_interface.py`).  LeRobot reports
the five arm joints normalized to [-100, 100] and the gripper to [0, 100];
the USD joint ranges are not symmetric, so each value is rescaled onto the
joint's real range in degrees:

    shoulder_pan  [-110, 110]   -> Rotation
    shoulder_lift [-100, 100]   -> Pitch
    elbow_flex    [-100,  90]   -> Elbow
    wrist_flex    [ -95,  95]   -> Wrist_Pitch
    wrist_roll    [-160, 160]   -> Wrist_Roll
    gripper       [ -10, 100]   -> Jaw

Packet fields:
    joint_positions   6 sim joint targets in radians (arm + Jaw)
    follower_positions  6 measured real follower joints in radians, when a
                       real arm is linked to the simulation; else omitted
    leader_positions  6 raw leader values, kept for dataset actions
    gripper_position  Jaw openness 0..1, for terminal status lines
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any


JOINT_COUNT = 6

SO101_SIM_JOINTS = (
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
)

SO101_LEADER_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# USD joint ranges in degrees (SO-ARM101-USD.usd authored limits, which the
# workshop mapping reproduces exactly).
SO101_JOINT_RANGES_DEG = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-160.0, 160.0),
    "gripper": (-10.0, 100.0),
}


def _finite_float(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def raw_to_deg(leader_joint: str, raw: float) -> float:
    """Map one raw leader value onto the joint's USD range in degrees."""
    lower, upper = SO101_JOINT_RANGES_DEG[leader_joint]
    raw = _finite_float(raw, leader_joint)
    if leader_joint == "gripper":
        normalized = min(1.0, max(0.0, raw / 100.0))
    else:
        normalized = min(1.0, max(0.0, (raw + 100.0) / 200.0))
    return lower + normalized * (upper - lower)


def raw_to_rad(leader_joint: str, raw: float) -> float:
    return math.radians(raw_to_deg(leader_joint, raw))


def rad_to_raw(leader_joint: str, radians: float) -> float:
    """Invert the mapping; used to report simulated joints as leader values."""
    lower, upper = SO101_JOINT_RANGES_DEG[leader_joint]
    degrees = math.degrees(_finite_float(radians, leader_joint))
    normalized = (degrees - lower) / (upper - lower)
    normalized = min(1.0, max(0.0, normalized))
    if leader_joint == "gripper":
        return normalized * 100.0
    return normalized * 200.0 - 100.0


def jaw_openness(jaw_rad: float) -> float:
    """Jaw angle in radians to a 0..1 openness (closed gripper -> 0)."""
    return min(1.0, max(0.0, rad_to_raw("gripper", jaw_rad) / 100.0))


def _leader_values(action: dict[str, Any], field: str) -> list[float]:
    values: list[float] = []
    for joint in SO101_LEADER_JOINTS:
        key = f"{joint}.pos"
        if key not in action:
            raise KeyError(f"Leader data is missing {key}")
        values.append(_finite_float(action[key], key))
    return values


def leader_action_to_payload(
    action: dict[str, Any],
    *,
    sequence: int,
    timestamp: float,
    follower_observation: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Convert one lerobot leader sample to the SO-101 Isaac UDP payload."""
    raw = _leader_values(action, "action")
    joint_positions = [
        raw_to_rad(joint, value)
        for joint, value in zip(SO101_LEADER_JOINTS, raw)
    ]
    payload: dict[str, Any] = {
        "sequence": int(sequence),
        "timestamp": _finite_float(timestamp, "timestamp"),
        "joint_positions": joint_positions,
        "gripper_position": jaw_openness(joint_positions[-1]),
        "leader_positions": raw,
        "source": "so101_leader",
    }
    if follower_observation is not None:
        follower_raw = _leader_values(follower_observation, "observation")
        payload["follower_positions"] = [
            raw_to_rad(joint, value)
            for joint, value in zip(SO101_LEADER_JOINTS, follower_raw)
        ]
        payload["follower_positions_raw"] = follower_raw
    return payload


def encode_payload(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


@dataclass(frozen=True)
class SO101Packet:
    joint_positions: tuple[float, ...]
    follower_positions: tuple[float, ...] | None
    leader_positions: tuple[float, ...]
    gripper_position: float
    sequence: int
    timestamp: float


def decode_payload(packet: bytes) -> SO101Packet:
    """Validate one SO-101 packet and return its fields."""
    payload = json.loads(packet.decode("utf-8"))
    sequence = int(payload["sequence"])
    timestamp = _finite_float(payload["timestamp"], "timestamp")

    def joint_list(name: str) -> list[float]:
        values = payload.get(name)
        if values is None:
            return []
        result = [
            _finite_float(value, f"{name}[{index}]")
            for index, value in enumerate(values)
        ]
        if len(result) != JOINT_COUNT:
            raise ValueError(
                f"{name} has length {len(result)}; expected {JOINT_COUNT}"
            )
        return result

    joint_positions = joint_list("joint_positions")
    if not joint_positions:
        raise ValueError("joint_positions is missing")
    follower_positions = joint_list("follower_positions") or None
    leader_positions = joint_list("leader_positions")
    if not leader_positions:
        # Older bridge revisions did not echo the raw leader values; the
        # recorded action vector then falls back to the mapped values.
        leader_positions = [
            rad_to_raw(joint, radians)
            for joint, radians in zip(SO101_LEADER_JOINTS, joint_positions)
        ]
    gripper_position = _finite_float(
        payload.get("gripper_position", 0.0), "gripper_position"
    )
    return SO101Packet(
        joint_positions=tuple(joint_positions),
        follower_positions=(
            tuple(follower_positions) if follower_positions else None
        ),
        leader_positions=tuple(leader_positions),
        gripper_position=gripper_position,
        sequence=sequence,
        timestamp=timestamp,
    )
