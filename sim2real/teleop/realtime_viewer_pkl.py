#!/usr/bin/env python3
"""
PKL file visualizer for retargeted/recorded robot poses.
"""

from __future__ import annotations

import os
import pickle
import time
from typing import TYPE_CHECKING

import numpy as np
from loop_rate_limiters import RateLimiter

if TYPE_CHECKING:
    from sim2real.config.robots import RobotCfg
    from sim2real.teleop.realtime_viewer import NativeRobotViewer, ViewerArgs


def run_pkl_viewer(
    args: ViewerArgs,
    robot_cfg: RobotCfg,
    viewer: NativeRobotViewer,
) -> None:
    if args.motion_path is None:
        raise ValueError("--motion_path is required when --motion_backend=pkl")

    if not os.path.exists(args.motion_path):
        raise FileNotFoundError(f"PKL motion path does not exist: {args.motion_path}")

    print(f"[viewer] Loading PKL motion from {args.motion_path}")
    with open(args.motion_path, "rb") as f:
        data = pickle.load(f)

    if not isinstance(data, dict):
        raise ValueError("PKL data must be a dictionary")

    # Required keys validation
    for key in ("root_pos", "root_rot", "dof_pos"):
        if key not in data:
            raise ValueError(f"PKL data is missing required key: {key}")

    root_pos = np.asarray(data["root_pos"], dtype=np.float32)
    root_rot = np.asarray(data["root_rot"], dtype=np.float32)
    dof_pos = np.asarray(data["dof_pos"], dtype=np.float32)

    num_frames = root_pos.shape[0]
    if num_frames == 0:
        raise RuntimeError("No frames found in PKL file")

    if root_rot.shape[0] != num_frames or dof_pos.shape[0] != num_frames:
        raise ValueError(
            f"Frame count mismatch: root_pos={num_frames}, "
            f"root_rot={root_rot.shape[0]}, dof_pos={dof_pos.shape[0]}"
        )

    # Validate dof dimension compatibility
    expected_dof_size = robot_cfg.qpos_size - 7
    if dof_pos.shape[1] != expected_dof_size:
        raise ValueError(
            f"Joint dimension mismatch for robot '{robot_cfg.name}': "
            f"file has {dof_pos.shape[1]} joints, expected {expected_dof_size}"
        )

    # Determine playback speed (fps)
    playback_fps = float(data.get("fps", args.viewer_hz))
    if playback_fps <= 0:
        playback_fps = float(args.viewer_hz)

    rate = RateLimiter(frequency=playback_fps, warn=False)
    print(
        f"[viewer] Playback backend=pkl, frames={num_frames}, "
        f"playback_fps={playback_fps:.2f} Hz"
    )

    frame_idx = 0
    try:
        while viewer.is_running():
            print(root_pos[0],"sssss")
            # Construct qpos: concatenate root_pos, root_rot, and dof_pos
            qpos = np.zeros(robot_cfg.qpos_size, dtype=np.float32)
            qpos[:3] = root_pos[frame_idx]
            qpos[3:7] = root_rot[frame_idx]
            qpos[7:] = dof_pos[frame_idx]
            print(root_rot[frame_idx],dof_pos[frame_idx])
            viewer.render(
                qpos,
                xrobot_frame=None,
                show_xrobot_frames=False,  # xrobot reference frames not available in pkl
            )

            frame_idx = (frame_idx + 1) % num_frames
            rate.sleep()
    except KeyboardInterrupt:
        print("KeyboardInterrupt, exiting viewer.")
