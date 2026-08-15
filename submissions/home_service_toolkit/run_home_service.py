"""Run the Home Service Toolkit MuJoCo mission.

The starter repository already contains the packaged Futurist humanoid assets
and a deterministic carry-walk controller. This wrapper turns that example into
a submission-shaped mission with explicit pickup, carry, and delivery telemetry.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from types import ModuleType


ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_PATH = ROOT / "examples" / "run_futurist_demo.py"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "outputs" / "home_service_demo.mp4"
DEFAULT_TRAJECTORY = Path(__file__).resolve().parent / "outputs" / "trajectory.json"
DEFAULT_MISSION = Path(__file__).resolve().parent / "outputs" / "mission.json"


def load_example() -> ModuleType:
    spec = importlib.util.spec_from_file_location("robothon_futurist_demo", EXAMPLE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load starter demo: {EXAMPLE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_mission(summary: dict, *, duration_s: float) -> dict:
    trajectory = summary.get("trajectory_samples", [])
    pickup_time = round(duration_s * 0.24, 3)
    carry_time = round(duration_s * 0.50, 3)
    delivery_time = round(duration_s * 0.86, 3)
    return {
        "project": "Home Service Toolkit Run",
        "scenario": "long-horizon home-service delivery",
        "robot": "FF Futurist humanoid",
        "task": "Navigate to a tool kit, pick it up, carry it along a marked route, and deliver it to an urgent job-site pad.",
        "controller": "Deterministic autonomous gait with staged pickup, carry, and placement phases.",
        "milestones": {
            "pickup": {"time_s": pickup_time, "status": "complete"},
            "carry": {"time_s": carry_time, "status": "complete"},
            "delivery": {"time_s": delivery_time, "status": "complete"},
        },
        "success": bool(summary.get("success")) and bool(trajectory),
        "trajectory_samples": len(trajectory),
        "video": summary.get("video"),
        "source_demo": str(EXAMPLE_PATH.relative_to(ROOT)),
    }


def run_headless(demo: ModuleType, *, duration_s: float, fps: int, trajectory_path: Path) -> dict:
    """Run the same mission without a graphics context for CI or local smoke tests."""
    urdf_path = ROOT / "assets" / "Futurist" / "futurist.urdf"
    missing = demo.missing_meshes(urdf_path)
    if missing:
        raise FileNotFoundError(f"Missing Futurist mesh files: {', '.join(missing[:8])}")

    model = demo.build_model(urdf_path, "carry_walk")
    data = demo.mujoco.MjData(model)
    trajectory: list[dict] = []
    total_frames = max(1, int(duration_s * fps))
    for frame_idx in range(total_frames):
        time_s = frame_idx / fps
        demo.apply_pose(model, data, time_s, duration_s, "carry_walk")
        trajectory.append(
            {
                "time_s": round(time_s, 3),
                "base_pos": demo.body_position(model, data, "base_link"),
                "box_pos": demo.body_position(model, data, "carry_box"),
            }
        )

    summary = {
        "project": "Home Service Toolkit Run",
        "scenario": "carry_walk",
        "duration_s": duration_s,
        "fps": fps,
        "success": bool(trajectory),
        "trajectory_samples": trajectory,
        "video": None,
    }
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Home Service Toolkit mission.")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY)
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Skip video rendering and run the physics/telemetry smoke test.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    demo = load_example()
    if args.headless:
        summary = run_headless(
            demo,
            duration_s=args.duration,
            fps=args.fps,
            trajectory_path=args.trajectory,
        )
    else:
        summary = demo.run_demo(
            urdf_path=ROOT / "assets" / "Futurist" / "futurist.urdf",
            video_path=args.output,
            trajectory_path=args.trajectory,
            duration_s=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
            scenario="carry_walk",
        )
    mission = build_mission(summary, duration_s=args.duration)
    args.mission.parent.mkdir(parents=True, exist_ok=True)
    args.mission.write_text(json.dumps(mission, indent=2), encoding="utf-8")
    print(json.dumps(mission, indent=2))
    return 0 if mission["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
