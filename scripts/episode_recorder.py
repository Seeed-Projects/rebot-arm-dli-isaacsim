#!/usr/bin/env python3
"""Disk-backed episode recorder with asynchronous LeRobot conversion."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
import uuid
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np


CAMERA_NAMES = ("front", "external")


class EpisodeRecorder:
    """Record images immediately and convert completed episodes off-process."""

    def __init__(
        self,
        *,
        dataset_root: Path,
        repo_id: str,
        task_name: str,
        fps: int,
        resolution: tuple[int, int],
        jpeg_quality: int,
        writer_python: Path,
        writer_script: Path,
    ) -> None:
        self.dataset_root = dataset_root.resolve()
        self.repo_id = repo_id
        self.task_name = task_name
        self.fps = int(fps)
        self.resolution = tuple(int(value) for value in resolution)
        self.jpeg_quality = int(jpeg_quality)
        # Preserve the venv's ``bin/python`` symlink. Resolving it would bypass
        # pyvenv.cfg and launch the base interpreter without LeRobot packages.
        self.writer_python = Path(os.path.abspath(str(writer_python)))
        self.writer_script = writer_script.resolve()
        self.staging_root = (
            self.dataset_root.parent / f".{self.dataset_root.name}_staging"
        )

        self.recording = False
        self.current_path: Path | None = None
        self.current_randomization: dict[str, Any] = {}
        self.actions: list[np.ndarray] = []
        self.states: list[np.ndarray] = []
        self.frame_count = 0
        self.image_std_sums = {name: 0.0 for name in CAMERA_NAMES}
        self.last_image_std = {name: 0.0 for name in CAMERA_NAMES}
        self.image_edge_std_sums = {name: 0.0 for name in CAMERA_NAMES}
        self.last_image_edge_std = {name: 0.0 for name in CAMERA_NAMES}
        self.started_at = 0.0
        self.pending: deque[Path] = deque()
        self.active_process: subprocess.Popen | None = None
        self.active_path: Path | None = None
        self.completed_episodes = 0
        self.failed_episodes = 0
        self.last_error: str | None = None

    def start(self, randomization: dict[str, Any]) -> None:
        if self.recording:
            return
        episode_id = (
            time.strftime("%Y%m%d_%H%M%S")
            + "_"
            + uuid.uuid4().hex[:8]
        )
        self.current_path = self.staging_root / f"episode_{episode_id}"
        for camera_name in CAMERA_NAMES:
            (self.current_path / "images" / camera_name).mkdir(
                parents=True, exist_ok=False
            )
        self.current_randomization = dict(randomization)
        self.actions = []
        self.states = []
        self.frame_count = 0
        self.image_std_sums = {name: 0.0 for name in CAMERA_NAMES}
        self.image_edge_std_sums = {name: 0.0 for name in CAMERA_NAMES}
        self.started_at = time.time()
        self.recording = True
        print(
            f"[record] Episode started at {self.fps} FPS -> {self.dataset_root}"
        )

    def capture(
        self,
        *,
        action: np.ndarray,
        state: np.ndarray,
        images: dict[str, np.ndarray],
    ) -> None:
        if not self.recording or self.current_path is None:
            return
        expected_width, expected_height = self.resolution
        for camera_name in CAMERA_NAMES:
            rgb = np.asarray(images[camera_name], dtype=np.uint8)
            if rgb.shape != (expected_height, expected_width, 3):
                raise ValueError(
                    f"{camera_name} image shape is {rgb.shape}; "
                    f"expected {(expected_height, expected_width, 3)}"
                )
            image_path = (
                self.current_path
                / "images"
                / camera_name
                / f"frame_{self.frame_count:06d}.jpg"
            )
            ok = cv2.imwrite(
                str(image_path),
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if not ok:
                raise RuntimeError(f"Unable to write image: {image_path}")
            self.image_std_sums[camera_name] += float(np.std(rgb))
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            self.image_edge_std_sums[camera_name] += float(
                np.std(cv2.Laplacian(gray, cv2.CV_32F))
            )
        self.actions.append(np.asarray(action, dtype=np.float32).copy())
        self.states.append(np.asarray(state, dtype=np.float32).copy())
        self.frame_count += 1

    def stop(
        self,
        *,
        success: bool,
        grasped: bool,
        max_object_lift_m: float,
    ) -> Path | None:
        if not self.recording or self.current_path is None:
            return None
        episode_path = self.current_path
        self.recording = False
        if self.frame_count == 0:
            print("[record-warn] The current episode has no frames and was canceled")
            shutil.rmtree(episode_path, ignore_errors=True)
            self._clear_current()
            return None

        np.save(
            episode_path / "actions.npy",
            np.stack(self.actions).astype(np.float32),
        )
        np.save(
            episode_path / "states.npy",
            np.stack(self.states).astype(np.float32),
        )
        self.last_image_std = {
            camera_name: total / self.frame_count
            for camera_name, total in self.image_std_sums.items()
        }
        self.last_image_edge_std = {
            camera_name: total / self.frame_count
            for camera_name, total in self.image_edge_std_sums.items()
        }
        metadata = {
            "schema_version": 1,
            "task": self.task_name,
            "repo_id": self.repo_id,
            "fps": self.fps,
            "resolution": list(self.resolution),
            "frames": self.frame_count,
            "started_at_unix": self.started_at,
            "stopped_at_unix": time.time(),
            "success": bool(success),
            "grasped": bool(grasped),
            "max_object_lift_m": float(max_object_lift_m),
            "camera_rgb_std_mean": self.last_image_std,
            "camera_laplacian_std_mean": self.last_image_edge_std,
            "randomization": self.current_randomization,
        }
        (episode_path / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.pending.append(episode_path)
        print(
            f"[record] Episode queued: {self.frame_count} frames, "
            f"success={bool(success)}"
        )
        self._clear_current()
        self.poll()
        return episode_path

    def cancel(self) -> None:
        if not self.recording or self.current_path is None:
            return
        cancelled_path = self.current_path
        self.recording = False
        shutil.rmtree(cancelled_path, ignore_errors=True)
        self._clear_current()
        print("[record] Current episode canceled; no dataset data was written")

    def _clear_current(self) -> None:
        self.current_path = None
        self.current_randomization = {}
        self.actions = []
        self.states = []
        self.frame_count = 0
        self.image_std_sums = {name: 0.0 for name in CAMERA_NAMES}
        self.image_edge_std_sums = {name: 0.0 for name in CAMERA_NAMES}
        self.started_at = 0.0

    @staticmethod
    def _clean_python_environment(python_path: Path) -> dict[str, str]:
        environment = os.environ.copy()
        environment.pop("PYTHONHOME", None)
        environment.pop("PYTHONPATH", None)
        environment["VIRTUAL_ENV"] = str(python_path.parent.parent)
        return environment

    def _start_next(self) -> None:
        if self.active_process is not None or not self.pending:
            return
        self.active_path = self.pending.popleft()
        command = [
            str(self.writer_python),
            str(self.writer_script),
            "--staging",
            str(self.active_path),
            "--dataset-root",
            str(self.dataset_root),
            "--repo-id",
            self.repo_id,
        ]
        self.active_process = subprocess.Popen(
            command,
            env=self._clean_python_environment(self.writer_python),
        )
        print(f"[dataset] Processing {self.active_path.name} in the background")

    def poll(self) -> None:
        if self.active_process is not None:
            return_code = self.active_process.poll()
            if return_code is not None:
                if return_code == 0:
                    self.completed_episodes += 1
                    print(
                        f"[dataset] Episode saved; total completed: "
                        f"{self.completed_episodes}"
                    )
                else:
                    self.failed_episodes += 1
                    self.last_error = (
                        f"Failed to convert {self.active_path}; exit code {return_code}"
                    )
                    print(f"[dataset-error] {self.last_error}")
                self.active_process = None
                self.active_path = None
        self._start_next()

    def finish(self, timeout_s: float = 180.0) -> bool:
        if self.recording:
            self.cancel()
        deadline = time.monotonic() + timeout_s
        while self.pending or self.active_process is not None:
            self.poll()
            if self.failed_episodes:
                return False
            if time.monotonic() >= deadline:
                self.last_error = "Timed out waiting for the background dataset task"
                return False
            time.sleep(0.1)
        return self.failed_episodes == 0
