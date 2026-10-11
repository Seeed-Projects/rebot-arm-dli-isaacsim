#!/usr/bin/env python3
"""Replay a recorded LeRobot episode over the Isaac teleoperation UDP port."""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from teleop_protocol import encode_payload, leader_action_to_payload


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "teleop_config.json"
DEFAULT_DR_CONFIG = PROJECT_ROOT / "config" / "domain_randomization.json"
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
]


def _parse_args() -> argparse.Namespace:
    dr_config = json.loads(DEFAULT_DR_CONFIG.read_text(encoding="utf-8"))
    recording = dr_config["recording"]
    dataset_root = Path(recording["dataset_root"])
    if not dataset_root.is_absolute():
        dataset_root = PROJECT_ROOT / dataset_root
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=dataset_root,
    )
    parser.add_argument("--repo-id", default=recording["repo_id"])
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--loop", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    host = args.host or str(config["udp"]["host"])
    port = args.port or int(config["udp"]["port"])
    dataset = LeRobotDataset(
        repo_id=args.repo_id,
        root=args.dataset_root.resolve(),
        episodes=[args.episode],
        download_videos=False,
    )
    # Newer lerobot releases load `hf_dataset` on access and dropped the
    # explicit loader; older ones need the call first.
    ensure_loaded = getattr(dataset, "_ensure_hf_dataset_loaded", None)
    if ensure_loaded is not None:
        ensure_loaded()
    rows = dataset.hf_dataset
    fps = float(dataset.fps)
    if len(rows) == 0:
        raise RuntimeError(f"Episode {args.episode} has no frames")

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    print(
        f"[replay] episode={args.episode}, frames={len(rows)}, "
        f"fps={fps:g} → udp://{host}:{port}"
    )
    try:
        while True:
            deadline = time.monotonic()
            for row in rows:
                action_values = np.asarray(row["action"], dtype=np.float64)
                leader_action = {
                    f"{name}.pos": float(value)
                    for name, value in zip(JOINT_NAMES, action_values)
                }
                payload = leader_action_to_payload(
                    leader_action,
                    config,
                    sequence=sequence,
                    timestamp=time.time(),
                )
                udp.sendto(encode_payload(payload), (host, port))
                sequence += 1
                deadline += 1.0 / fps
                time.sleep(max(0.0, deadline - time.monotonic()))
            if not args.loop:
                break
    finally:
        udp.close()
    print("[replay] Complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
