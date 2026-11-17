#!/usr/bin/env python3
"""Record a Hunter180 AMP checkpoint walking in MuJoCo, without a GUI.

Run with the AMP_mjlab Python environment. The default checkpoint comes from
this deployment's policy manifest; training files are only read.
"""

import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-repo", type=Path, default=ROOT.parent / "AMP_mjlab")
    parser.add_argument("--run-dir", type=Path, help="Training run directory; defaults to the deployed policy's run")
    parser.add_argument("--checkpoint", help="Checkpoint path, model_N.pt, or latest; defaults to the deployed checkpoint")
    parser.add_argument("--output", type=Path, help="Output MP4; defaults to log/videos/amp_walk_<timestamp>.mp4")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--vx", type=float, default=0.6, help="Forward command (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral command (m/s)")
    parser.add_argument("--wz", type=float, default=0.0, help="Yaw command (rad/s)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    if not all(math.isfinite(value) for value in (args.vx, args.vy, args.wz)):
        parser.error("Velocity commands must be finite")
    if min(args.width, args.height) <= 0 or args.width % 2 or args.height % 2:
        parser.error("Video width and height must be positive even integers")
    return args


def resolve_paths(args):
    manifest = json.loads((ROOT / "src/ec_controller/config/policy/hunter180_amp.json").read_text())
    repo = args.training_repo.expanduser().resolve()
    if not (repo / "src/tasks/amp_loco").is_dir():
        raise FileNotFoundError(f"AMP_mjlab training repository not found: {repo}")
    run = (args.run_dir.expanduser().resolve() if args.run_dir else
           repo / "logs/rsl_rl/hunter180_amp_baseline" / manifest["run"])
    choice = args.checkpoint or manifest["checkpoint"]
    if choice == "latest":
        candidates = [path for path in run.glob("model_*.pt") if path.stem[6:].isdigit()]
        if not candidates:
            raise FileNotFoundError(f"No model_N.pt checkpoints found in {run}")
        checkpoint = max(candidates, key=lambda path: int(path.stem[6:]))
    else:
        checkpoint = Path(choice).expanduser()
        if not checkpoint.is_absolute():
            checkpoint = run / checkpoint
    checkpoint = checkpoint.resolve(strict=True)
    checkpoint_hash = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    if args.checkpoint is None and args.run_dir is None:
        if checkpoint_hash != manifest["checkpoint_sha256"]:
            raise ValueError("Checkpoint SHA-256 differs from the deployed policy manifest")
    output = (args.output or ROOT / "log/videos" /
              f"amp_walk_{checkpoint.stem}_{datetime.now():%Y%m%d_%H%M%S}.mp4").expanduser().resolve()
    if output.suffix.lower() != ".mp4":
        raise ValueError("--output must end in .mp4")
    if output.exists() or output.with_suffix(".json").exists():
        raise FileExistsError(f"Recording already exists: {output}")
    return repo, checkpoint, checkpoint_hash, output


def record(args):
    repo, checkpoint, checkpoint_hash, output = resolve_paths(args)
    # Set the offscreen backend before importing mjlab / MuJoCo.
    os.environ.setdefault("MUJOCO_GL", "egl")
    nvidia_egl = Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    if os.environ["MUJOCO_GL"] == "egl" and nvidia_egl.exists():
        os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", str(nvidia_egl))
    sys.path.insert(0, str(repo))
    os.chdir(repo)

    import imageio.v2 as imageio
    import numpy as np
    import torch
    import mjlab.tasks  # noqa: F401
    import src.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv, mdp
    from mjlab.managers.event_manager import EventTermCfg
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
    from mjlab.utils.torch import configure_torch_backends

    configure_torch_backends()
    task = "Hunter180-AMP-Flat"
    cfg = load_env_cfg(task, play=True)
    agent_cfg = load_rl_cfg(task)
    cfg.seed = args.seed
    cfg.scene.num_envs = 1
    cfg.viewer.width, cfg.viewer.height = args.width, args.height
    cfg.viewer.distance, cfg.viewer.elevation, cfg.viewer.azimuth = 3.5, -15.0, 135.0

    # Start from the default standing pose for a walking demonstration, instead
    # of randomly sampling a walking / recovery reference frame at reset.
    cfg.events.pop("init_motion_loader", None)
    cfg.events["reset_from_motion"] = EventTermCfg(func=mdp.reset_scene_to_default, mode="reset")
    twist = cfg.commands["twist"]
    twist.heading_command = False
    twist.rel_heading_envs = twist.rel_standing_envs = 0.0
    twist.init_velocity_prob = 0.0
    twist.ranges.heading = None
    twist.ranges.lin_vel_x = (args.vx, args.vx)
    twist.ranges.lin_vel_y = (args.vy, args.vy)
    twist.ranges.ang_vel_z = (args.wz, args.wz)
    twist.debug_vis = False

    print(f"Checkpoint: {checkpoint}", flush=True)
    print(f"Command: vx={args.vx}, vy={args.vy} m/s, wz={args.wz} rad/s", flush=True)
    raw_env = ManagerBasedRlEnv(cfg=cfg, device=args.device, render_mode="rgb_array")
    try:
        env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
        runner_cls = load_runner_cls(task)
        if runner_cls is None:
            raise RuntimeError(f"No policy runner registered for {task}")
        runner = runner_cls(env, asdict(agent_cfg), device=args.device)
        runner.load(str(checkpoint), load_optimizer=False)
        policy = runner.get_inference_policy(device=args.device)
        obs = env.get_observations()
        fps = 1.0 / raw_env.step_dt
        steps = max(1, round(args.seconds * fps))
        robot = raw_env.scene["robot"]
        start_position = robot.data.root_link_pos_w[0].detach().cpu().tolist()
        minimum_height = start_position[2]
        resets = 0
        output.parent.mkdir(parents=True, exist_ok=True)
        with imageio.get_writer(str(output), fps=fps, codec="libx264", quality=8,
                                macro_block_size=2) as writer, torch.inference_mode():
            for step in range(steps):
                actions = policy(obs)
                if not torch.isfinite(actions).all():
                    raise ValueError(f"Policy produced non-finite actions at step {step}")
                obs, _, dones, _ = env.step(actions)
                resets += int(dones[0].item())
                minimum_height = min(minimum_height, float(robot.data.root_link_pos_w[0, 2]))
                frame = raw_env.render()
                if frame is None:
                    raise RuntimeError("MuJoCo offscreen renderer returned no frame")
                if frame.ndim == 4:
                    frame = frame[0]
                if frame.dtype != np.uint8:
                    frame = (np.clip(frame, 0, 1) * 255).astype(np.uint8)
                writer.append_data(frame)
                if (step + 1) % max(1, steps // 4) == 0 or step + 1 == steps:
                    print(f"Recorded {step + 1}/{steps} frames", flush=True)
        metadata = {
            "task": task, "checkpoint": str(checkpoint), "checkpoint_sha256": checkpoint_hash,
            "velocity_command": {"vx": args.vx, "vy": args.vy, "wz": args.wz},
            "seed": args.seed, "device": args.device, "fps": fps, "frames": steps,
            "duration_seconds": steps / fps, "width": args.width, "height": args.height,
            "initial_state": "default_standing_pose", "episode_resets": resets,
            "minimum_base_height_m": minimum_height, "start_position_m": start_position,
            "end_position_m": robot.data.root_link_pos_w[0].detach().cpu().tolist(),
        }
        output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Saved: {output}\nEpisode resets: {resets}", flush=True)
    finally:
        raw_env.close()


if __name__ == "__main__":
    record(parse_args())
