#!/usr/bin/env python3
"""Convert one staged reBot teleoperation episode to LeRobot Dataset v3."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset


JOINT_NAMES = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_yaw.pos",
    "wrist_roll.pos",
    "gripper.pos",
]
CAMERA_NAMES = ("front", "external")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staging", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    return parser.parse_args()


def _features(
    *, fps: int, width: int, height: int
) -> dict[str, dict]:
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (7,),
            "names": JOINT_NAMES,
        },
        "observation.images.front": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.images.external": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
        "action": {
            "dtype": "float32",
            "shape": (7,),
            "names": JOINT_NAMES,
        },
    }


def _load_or_create(
    *,
    root: Path,
    repo_id: str,
    fps: int,
    width: int,
    height: int,
) -> LeRobotDataset:
    info_path = root / "meta" / "info.json"
    if info_path.is_file():
        dataset = LeRobotDataset(
            repo_id=repo_id,
            root=root,
            batch_encoding_size=1,
            vcodec="h264",
        )
        if int(dataset.fps) != fps:
            raise ValueError(
                f"Existing dataset FPS={dataset.fps}, current episode FPS={fps}"
            )
        return dataset
    if root.exists():
        raise FileExistsError(
            f"Dataset directory exists but is not a valid LeRobot dataset: {root}"
        )
    return LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=_features(fps=fps, width=width, height=height),
        root=root,
        robot_type="seeed_b601_rs_follower",
        use_videos=True,
        batch_encoding_size=1,
        vcodec="h264",
    )


def main() -> int:
    args = _parse_args()
    staging = args.staging.resolve()
    dataset_root = args.dataset_root.resolve()
    metadata = json.loads(
        (staging / "metadata.json").read_text(encoding="utf-8")
    )
    actions = np.load(staging / "actions.npy")
    states = np.load(staging / "states.npy")
    frame_count = int(metadata["frames"])
    width, height = [int(value) for value in metadata["resolution"]]
    fps = int(metadata["fps"])
    if actions.shape != (frame_count, 7):
        raise ValueError(f"Invalid action shape: {actions.shape}")
    if states.shape != (frame_count, 7):
        raise ValueError(f"Invalid state shape: {states.shape}")

    dataset = _load_or_create(
        root=dataset_root,
        repo_id=args.repo_id,
        fps=fps,
        width=width,
        height=height,
    )
    episode_index = int(dataset.meta.total_episodes)
    try:
        for frame_index in range(frame_count):
            images: dict[str, np.ndarray] = {}
            for camera_name in CAMERA_NAMES:
                image_path = (
                    staging
                    / "images"
                    / camera_name
                    / f"frame_{frame_index:06d}.jpg"
                )
                with Image.open(image_path) as image:
                    images[camera_name] = np.asarray(
                        image.convert("RGB"), dtype=np.uint8
                    )
            dataset.add_frame(
                {
                    "action": actions[frame_index],
                    "observation.state": states[frame_index],
                    "observation.images.front": images["front"],
                    "observation.images.external": images["external"],
                    "task": metadata["task"],
                }
            )
        dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
    except Exception:
        try:
            dataset.finalize()
        except Exception:
            pass
        raise

    sidecar_dir = dataset_root / "episode_metadata"
    sidecar_dir.mkdir(parents=True, exist_ok=True)
    metadata["episode_index"] = episode_index
    (sidecar_dir / f"episode_{episode_index:06d}.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    shutil.rmtree(staging)
    print(
        f"[dataset-writer] episode={episode_index}, frames={frame_count}, "
        f"root={dataset_root}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
