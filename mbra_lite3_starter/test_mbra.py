#!/usr/bin/env python3
"""Offline smoke test of UCLA-VAIL MBRA_PG_Official (trajectory only).

This script NEVER publishes robot commands. It calls the official Python wrapper,
not the raw ONNX interface. Download the official files as described in README.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image


def model_dir_from_script() -> Path:
    return Path(__file__).resolve().parent / "MBRA_PG_Official"


def load_navigator(model_dir: Path):
    # Torch is imported BEFORE creating the ONNX session, so its CUDA/cuDNN
    # libraries can be available to ONNX Runtime on typical Linux installs.
    import torch
    import onnxruntime as ort

    inference_py = model_dir / "inference.py"
    onnx_path = model_dir / "mbra.onnx"
    info_path = model_dir / "model_info.yaml"
    for required in (inference_py, onnx_path, info_path):
        if not required.is_file():
            raise FileNotFoundError(f"Required official model file not found: {required}")

    spec = importlib.util.spec_from_file_location("mbra_pg_official_inference", inference_py)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not import official inference.py from {inference_py}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # NOTE: the official wrapper currently sets CUDA and CPU ORT providers
    # internally, independently of its `device` argument. We verify the
    # effective session providers AFTER construction to catch CPU fallback.
    navigator = module.MBRAPGNavigator(
        onnx_path=str(onnx_path),
        device="cuda" if torch.cuda.is_available() else "cpu",
    )
    providers = navigator._session.get_providers()
    print(f"PyTorch: {torch.__version__}")
    print(f"PyTorch CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"ONNX Runtime: {ort.__version__}")
    print(f"ORT available providers: {ort.get_available_providers()}")
    print(f"Model session providers: {providers}")
    return navigator, providers, torch.cuda.is_available()


def make_observation(images: Path | None, seed: int) -> np.ndarray:
    """Return float32 RGB observation (1, 6, 3, H, W) in [0,1]."""
    if images is None:
        print("Using RANDOM images: infrastructure test only, not navigation validation.")
        return np.random.default_rng(seed).random(
            (1, 6, 3, 96, 96), dtype=np.float32
        )

    allowed = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
    if not images.is_dir():
        raise NotADirectoryError(f"Image folder does not exist: {images}")
    files = sorted(p for p in images.iterdir() if p.suffix.lower() in allowed)
    if len(files) != 6:
        raise ValueError(
            f"--images needs exactly 6 image files (oldest to newest by filename); got {len(files)}"
        )
    frames = []
    target_size = None
    for path in files:
        with Image.open(path) as im:
            im = im.convert("RGB")
            if target_size is None:
                target_size = im.size
            elif im.size != target_size:
                raise ValueError(
                    f"Images must have same size: {path.name} has {im.size}; expected {target_size}"
                )
            rgb = np.asarray(im, dtype=np.uint8)
        frames.append(np.moveaxis(rgb, -1, 0).astype(np.float32) / 255.0)
    print("Image sequence (oldest -> newest):", ", ".join(f.name for f in files))
    return np.stack(frames, axis=0)[None, ...]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=model_dir_from_script())
    parser.add_argument("--images", type=Path, default=None,
                        help="Directory with exactly six RGB images named in time order")
    parser.add_argument("--goal-x", type=float, default=3.0,
                        help="Local goal x, meters, forward positive")
    parser.add_argument("--goal-y", type=float, default=0.0,
                        help="Local goal y, meters, left positive")
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--csv", type=Path, default=Path("trajectory.csv"))
    parser.add_argument("--allow-cpu", action="store_true",
                        help="Allow CPU fallback for diagnosis (GPU required by default)")
    args = parser.parse_args()
    if not all(map(math.isfinite, (args.goal_x, args.goal_y))):
        parser.error("Goal coordinates must be finite numbers")
    if args.runs < 1 or args.warmup < 0:
        parser.error("--runs must be >=1 and --warmup >=0")

    navigator, providers, torch_cuda = load_navigator(args.model_dir.resolve())
    if not args.allow_cpu and (not torch_cuda or "CUDAExecutionProvider" not in providers):
        raise RuntimeError(
            "GPU is required but not available in the actual inference session. "
            "Fix CUDA/cuDNN/ORT, or use --allow-cpu only for debugging."
        )

    obs = make_observation(args.images, args.seed)
    goal = np.array([args.goal_x, args.goal_y], dtype=np.float32)
    print("Observation shape:", obs.shape, "dtype:", obs.dtype)
    for _ in range(args.warmup):
        navigator.inference_trajectory(obs, goal_xy=goal)

    times = []
    trajectory = scores = None
    for _ in range(args.runs):
        begin = time.perf_counter()
        trajectory, scores = navigator.inference_trajectory(obs, goal_xy=goal)
        times.append((time.perf_counter() - begin) * 1000.0)

    if trajectory is None or scores is None:
        raise RuntimeError("No trajectory generated")
    trajectory = np.asarray(trajectory)
    scores = np.asarray(scores)
    print("Trajectory shape:", trajectory.shape)
    print("Scores shape:", scores.shape)
    if trajectory.shape != (1, 1, 8, 2):
        raise ValueError(f"Unexpected official wrapper trajectory shape: {trajectory.shape}")
    if not np.isfinite(trajectory).all():
        raise ValueError("Trajectory contains NaN or Inf")

    xy = trajectory[0, 0]
    print(f"Latency (including preprocessing) ms: "
          f"p50={np.percentile(times, 50):.2f}, "
          f"p95={np.percentile(times, 95):.2f}, "
          f"mean={np.mean(times):.2f}")
    print("8 local waypoints [x_forward_m, y_left_m]:")
    print(np.array2string(xy, precision=3))
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x_forward_m", "y_left_m"])
        writer.writerows((i, float(x), float(y)) for i, (x, y) in enumerate(xy, start=1))
    print("Saved:", args.csv.resolve())
    if not args.allow_cpu:
        print("CUDA EP registered in session; this alone does not prove every op ran on GPU.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        sys.exit(1)
