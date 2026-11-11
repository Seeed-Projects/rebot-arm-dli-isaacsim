#!/usr/bin/env python3
"""Read an SO-101 leader, drive the real SO-101 follower, and stream to Isaac.

Modes:
- real: connect the SO-101 leader (and, when a follower port is available,
  also the real SO-101 follower).  Each cycle the leader action is sent to
  the real arm and both the commanded leader state and the follower's
  measured joints are published over UDP, so the simulated SO-ARM101 in
  ``so101_teleop.py`` can mirror the real arm (sim_source=follower) or the
  leader (sim_source=leader).
- mock: a fixed scripted pose timeline for offline regression; no hardware
  is touched.

This script runs inside the LeRobot virtual environment (Python 3.12); the
Isaac Sim application spawns it, mirroring ``leader_bridge.py``.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import socket
import sys
import time
from pathlib import Path

from so101_protocol import encode_payload, leader_action_to_payload


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "teleop_config_so101.json"
_running = True


def _stop(_signum, _frame) -> None:
    global _running
    _running = False


signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--leader-port", help="SO-101 leader serial port, for example /dev/ttyUSB0"
    )
    parser.add_argument(
        "--follower-port",
        help="Real SO-101 follower serial port, for example /dev/ttyUSB1",
    )
    parser.add_argument(
        "--no-follower",
        action="store_true",
        help="Read the leader only; do not drive a real follower arm",
    )
    parser.add_argument(
        "--sim-source",
        choices=("follower", "leader"),
        default=None,
        help="Which measurements the simulation should mirror (default: config)",
    )
    parser.add_argument("--host")
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--rate", type=float)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Explicit test mode: send a fixed safe timeline without hardware",
    )
    parser.add_argument(
        "--mock-profile",
        choices=("hold", "grasp"),
        default="hold",
        help="Mock test data; only used with --mock",
    )
    return parser.parse_args()


def _resolve_serial_port(
    requested: str | None, default_port: str, role: str, exclude: set[str]
) -> str | None:
    if requested:
        return requested
    # SO-101 controller boards enumerate as USB-CDC (ttyACM) as well as
    # classic USB-UART (ttyUSB), so scan both.
    devices = sorted(
        glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    )
    available = [d for d in devices if d not in exclude]
    if default_port in available:
        return default_port
    if not available:
        return None
    if len(available) == 1:
        return available[0]
    raise RuntimeError(
        f"Multiple serial ports detected for the SO-101 {role}: {available}. "
        f"Select one explicitly with --{role}-port."
    )


def _lerp(start: float, end: float, alpha: float) -> float:
    alpha = min(1.0, max(0.0, alpha))
    alpha = alpha * alpha * (3.0 - 2.0 * alpha)
    return start + alpha * (end - start)


def _mock_action(elapsed: float, profile: str) -> dict[str, float]:
    """A scripted leader timeline in lerobot raw units (see so101_protocol).

    The timeline loops every 7 s (2.5 s hold between sweeps) so repeated
    auto-recorded episodes each contain the same scripted motion.
    """
    elapsed = elapsed % 7.0
    action = {
        "shoulder_pan.pos": 0.0,
        "shoulder_lift.pos": -25.0,
        "elbow_flex.pos": 25.0,
        "wrist_flex.pos": 25.0,
        "wrist_roll.pos": 0.0,
        "gripper.pos": 70.0,
    }
    if profile != "grasp":
        return action
    if elapsed < 0.5:
        pass
    elif elapsed < 2.0:
        alpha = (elapsed - 0.5) / 1.5
        action["shoulder_lift.pos"] = _lerp(-25.0, 35.0, alpha)
        action["elbow_flex.pos"] = _lerp(25.0, -35.0, alpha)
    elif elapsed < 3.0:
        alpha = (elapsed - 2.0) / 1.0
        action["shoulder_lift.pos"] = 35.0
        action["elbow_flex.pos"] = -35.0
        action["wrist_flex.pos"] = _lerp(25.0, 0.0, alpha)
        action["gripper.pos"] = _lerp(70.0, 10.0, alpha)
    elif elapsed < 4.5:
        alpha = (elapsed - 3.0) / 1.5
        action["shoulder_lift.pos"] = _lerp(35.0, -5.0, alpha)
        action["elbow_flex.pos"] = _lerp(-35.0, 40.0, alpha)
        action["wrist_flex.pos"] = 0.0
        action["gripper.pos"] = 10.0
    else:
        action["shoulder_lift.pos"] = -5.0
        action["elbow_flex.pos"] = 40.0
        action["wrist_flex.pos"] = 0.0
        action["gripper.pos"] = 10.0
    return action


def _import_so101_classes() -> tuple[type, type, type, type]:
    """Import the SO-101 leader/follower classes across lerobot variants.

    The Seeed fork keeps them in the so_leader/so_follower modules; upstream
    lerobot and the workshop image expose so101_leader/so101_follower.
    """
    try:
        from lerobot.teleoperators.so101_leader import (  # type: ignore
            SO101Leader,
            SO101LeaderConfig,
        )
        from lerobot.robots.so101_follower import (  # type: ignore
            SO101Follower,
            SO101FollowerConfig,
        )
        return SO101Leader, SO101LeaderConfig, SO101Follower, SO101FollowerConfig
    except ImportError:
        from lerobot.teleoperators.so_leader.so_leader import (  # type: ignore
            SOLeader,
            SOLeaderTeleopConfig,
        )
        from lerobot.robots.so_follower.so_follower import (  # type: ignore
            SOFollower,
            SOFollowerRobotConfig,
        )
        return SOLeader, SOLeaderTeleopConfig, SOFollower, SOFollowerRobotConfig


def main() -> int:
    args = _parse_args()
    config = json.loads(args.config.resolve().read_text(encoding="utf-8"))
    udp = config["udp"]
    host = args.host or udp["host"]
    udp_port = args.udp_port or int(udp["port"])
    rate = args.rate or float(udp["send_hz"])
    if rate <= 0:
        raise ValueError("The send rate must be greater than zero")
    sim_source = args.sim_source or str(
        config.get("follower", {}).get("sim_source", "follower")
    )

    leader = None
    follower = None
    if args.mock:
        print("[so101-leader] MOCK sender enabled; real hardware is not being accessed")
    else:
        leader_port = _resolve_serial_port(
            args.leader_port,
            str(config["leader"]["default_port"]),
            "leader",
            exclude=set(),
        )
        if leader_port is None:
            raise RuntimeError(
                "No /dev/ttyUSB* or /dev/ttyACM* device was detected for the "
                "SO-101 leader. Connect the leader arm, then run "
                "`ls /dev/ttyUSB* /dev/ttyACM*`."
            )
        if not os.access(leader_port, os.R_OK | os.W_OK):
            raise PermissionError(
                f"The current user cannot read or write the serial port: {leader_port}\n"
                f"Run this command first: sudo chmod 666 {leader_port}"
            )
        try:
            SO101Leader, SO101LeaderConfig, SO101Follower, SO101FollowerConfig = (
                _import_so101_classes()
            )
        except ImportError as exc:
            raise RuntimeError(
                "The current Python environment does not provide the SO-101 "
                "leader/follower classes. Run this bridge with the LeRobot "
                "virtual environment configured in "
                "config/teleop_config_so101.json."
            ) from exc

        leader_config = SO101LeaderConfig(
            port=leader_port,
            id=str(config["leader"]["id"]),
        )
        leader = SO101Leader(leader_config)
        # The SO-101 is_calibrated check reads the motors' calibration
        # registers over the bus, so the port must be open first (the
        # reBot leader checks a file instead, which is why its bridge
        # could check before connecting).
        leader.connect(calibrate=False)
        if not leader.is_calibrated:
            leader.disconnect()
            raise RuntimeError(
                "The leader is not calibrated. Follow the workshop guide "
                "and run "
                "`lerobot-calibrate --teleop.type=so101_leader "
                f"--teleop.port={leader_port} "
                f"--teleop.id={config['leader']['id']}`"
            )
        print(
            f"[so101-leader] Connected to {leader_port}; "
            f"calibration ID={config['leader']['id']}"
        )

        follower_cfg = config.get("follower", {})
        follower_port = None
        if not args.no_follower and bool(follower_cfg.get("enabled", False)):
            follower_port = _resolve_serial_port(
                args.follower_port,
                str(follower_cfg.get("default_port", "")),
                "follower",
                exclude={leader_port},
            )
        if not args.no_follower and follower_port is None and args.follower_port:
            follower_port = args.follower_port
        if not args.no_follower and follower_port is not None:
            if not os.access(follower_port, os.R_OK | os.W_OK):
                raise PermissionError(
                    f"The current user cannot read or write the serial port: "
                    f"{follower_port}\n"
                    f"Run this command first: sudo chmod 666 {follower_port}"
                )
            follower_config = SO101FollowerConfig(
                port=follower_port,
                id=str(follower_cfg.get("id", "so101_follower")),
            )
            follower = SO101Follower(follower_config)
            follower.connect(calibrate=False)
            if not follower.is_calibrated:
                follower.disconnect()
                leader.disconnect()
                raise RuntimeError(
                    "The follower is not calibrated. Run "
                    "`lerobot-calibrate --robot.type=so101_follower "
                    f"--robot.port={follower_port} "
                    f"--teleop.id={follower_cfg.get('id', 'so101_follower')}`"
                )
            print(
                f"[so101-follower] Real arm linked on {follower_port}; "
                "the simulation will mirror its measured joints"
            )
        else:
            print(
                "[so101-follower] No follower port given or found; "
                "running sim-only teleoperation from the leader"
            )

    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / rate
    sequence = 0
    started_at = time.perf_counter()
    next_tick = time.perf_counter()
    print(
        f"[so101-leader] Sending to udp://{host}:{udp_port} at {rate:.1f} Hz "
        f"(sim_source={sim_source})"
    )
    try:
        while _running:
            action = (
                _mock_action(time.perf_counter() - started_at, args.mock_profile)
                if args.mock
                else dict(leader.get_action())
            )
            follower_observation = None
            if follower is not None:
                follower.send_action(action)
                if sim_source == "follower":
                    observation = dict(follower.get_observation())
                    follower_observation = {
                        key: value
                        for key, value in observation.items()
                        if key.endswith(".pos")
                    }
            payload = leader_action_to_payload(
                action,
                sequence=sequence,
                timestamp=time.time(),
                follower_observation=follower_observation,
            )
            output.sendto(encode_payload(payload), (host, udp_port))
            if sequence % max(int(rate), 1) == 0:
                q_deg = [
                    round(value * 180.0 / 3.141592653589793, 2)
                    for value in payload["joint_positions"]
                ]
                follow = (
                    "follower="
                    + str(
                        [
                            round(value * 180.0 / 3.141592653589793, 1)
                            for value in payload.get("follower_positions", [])
                        ]
                    )
                    if follower is not None
                    else "sim-only"
                )
                print(
                    f"[so101-leader] seq={sequence} q_sim_deg={q_deg} {follow}"
                )
            sequence += 1
            next_tick += period
            time.sleep(max(0.0, next_tick - time.perf_counter()))
    finally:
        output.close()
        if follower is not None and follower.is_connected:
            follower.disconnect()
            print("[so101-follower] Real arm disconnected")
        if leader is not None and leader.is_connected:
            leader.disconnect()
            print("[so101-leader] Stopped and closed the serial port")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[so101-leader-error] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
