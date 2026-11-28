#!/usr/bin/env python3
"""Move offline IMU linear acceleration from the sensor point to base_link.

The input format is the one used by rgbd_inertial_tum:

    timestamp gx gy gz ax ay az

Angular velocity and acceleration axes must already be expressed in base_link.
``--offset-base-to-sensor`` is the vector from the base_link origin to the
physical IMU measurement point, expressed in base_link.  For a rigid body:

    a_sensor = a_base + alpha x r + omega x (omega x r)

so this tool writes:

    a_base = a_sensor - alpha x r - omega x (omega x r)

Gravity is unchanged because the two frames have the same orientation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compensate IMU lever-arm acceleration to base_link"
    )
    parser.add_argument("input_imu", type=Path, help="Input imu.txt")
    parser.add_argument("output_imu", type=Path, help="Output compensated imu.txt")
    parser.add_argument(
        "--offset-base-to-sensor",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.005, 0.0, 0.0),
        help="Sensor position relative to base_link in metres (default: 0.005 0 0)",
    )
    parser.add_argument(
        "--savgol-window",
        type=int,
        default=11,
        help="Odd gyro smoothing window used before differentiation (default: 11)",
    )
    parser.add_argument(
        "--savgol-order",
        type=int,
        default=3,
        help="Savitzky-Golay polynomial order (default: 3)",
    )
    return parser.parse_args()


def load_imu(path: Path) -> np.ndarray:
    data = np.loadtxt(path, comments="#", dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 7:
        raise ValueError(f"expected 7 columns, got {data.shape[1]}")
    if len(data) < 5:
        raise ValueError("at least 5 IMU samples are required")
    if not np.all(np.isfinite(data)):
        raise ValueError("input contains non-finite values")
    if np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError("timestamps must be strictly increasing")
    return data


def smooth_gyro(gyro: np.ndarray, window: int, order: int) -> np.ndarray:
    if window < 3 or window % 2 == 0:
        raise ValueError("--savgol-window must be an odd integer >= 3")
    if order < 1 or order >= window:
        raise ValueError("--savgol-order must be >= 1 and smaller than the window")
    if window > len(gyro):
        raise ValueError("Savitzky-Golay window is longer than the input")
    try:
        from scipy.signal import savgol_filter
    except ImportError as error:
        raise RuntimeError(
            "scipy is required for stable angular-acceleration estimation"
        ) from error
    return savgol_filter(gyro, window, order, axis=0, mode="interp")


def main() -> int:
    args = parse_args()
    data = load_imu(args.input_imu)
    timestamps = data[:, 0]
    gyro = data[:, 1:4]
    acceleration_sensor = data[:, 4:7]
    offset = np.asarray(args.offset_base_to_sensor, dtype=np.float64)

    gyro_filtered = smooth_gyro(gyro, args.savgol_window, args.savgol_order)
    angular_acceleration = np.gradient(
        gyro_filtered, timestamps, axis=0, edge_order=2
    )
    tangential = np.cross(angular_acceleration, offset)
    centripetal = np.cross(gyro_filtered, np.cross(gyro_filtered, offset))
    correction = tangential + centripetal
    acceleration_base = acceleration_sensor - correction

    output = data.copy()
    output[:, 4:7] = acceleration_base
    args.output_imu.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "timestamp gx gy gz ax ay az\n"
        "linear acceleration shifted from physical IMU point to base_link; "
        f"r_base_to_sensor={offset.tolist()} m; "
        f"savgol_window={args.savgol_window}; savgol_order={args.savgol_order}"
    )
    np.savetxt(
        args.output_imu,
        output,
        fmt="%.9f %.12g %.12g %.12g %.12g %.12g %.12g",
        header=header,
        comments="# ",
    )

    norm = np.linalg.norm(correction, axis=1)
    print(f"Samples: {len(output)}")
    print(f"Duration: {timestamps[-1] - timestamps[0]:.6f} s")
    print(f"Offset base->sensor: {offset.tolist()} m")
    print(
        "Acceleration correction norm [m/s^2]: "
        f"median={np.median(norm):.9f}, "
        f"p95={np.percentile(norm, 95):.9f}, "
        f"p99={np.percentile(norm, 99):.9f}, max={np.max(norm):.9f}"
    )
    print(f"Output: {args.output_imu}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
