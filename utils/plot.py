#!/usr/bin/env python3

import argparse
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np


def load_xy(path: Path, ground_truth: bool = False):
    data = np.atleast_2d(np.loadtxt(path))
    if data.shape[1] < 3:
        raise ValueError(f"{path} must contain time, north, and east columns")

    if ground_truth and data.shape[1] < 4:
        raise ValueError(f"{path} must contain at least four columns")
    if ground_truth and data.shape[1] != 8:
        lat, lon, alt = data[:, 1], data[:, 2], data[:, 3]
        a, f = 6378137.0, 1.0 / 298.257223563
        e2 = f * (2.0 - f)
        lat_r, lon_r = np.deg2rad(lat), np.deg2rad(lon)
        radius = a / np.sqrt(1.0 - e2 * np.sin(lat_r) ** 2)
        x = (radius + alt) * np.cos(lat_r) * np.cos(lon_r)
        y = (radius + alt) * np.cos(lat_r) * np.sin(lon_r)
        z = (radius * (1.0 - e2) + alt) * np.sin(lat_r)
        lat0, lon0 = lat_r[0], lon_r[0]
        dx, dy, dz = x - x[0], y - y[0], z - z[0]
        north = -np.sin(lat0) * np.cos(lon0) * dx
        north -= np.sin(lat0) * np.sin(lon0) * dy
        north += np.cos(lat0) * dz
        east = -np.sin(lon0) * dx + np.cos(lon0) * dy
    else:
        north, east = data[:, 1], data[:, 2]

    return data[:, 0] - data[0, 0], north - north[0], east - east[0]


def output_file(path: Path) -> Path:
    return path / "Body_traj.txt" if path.is_dir() else path


def default_ground_truth(traj: Path) -> Optional[Path]:
    name = traj.parent.name
    candidates = (
        Path("datasets/gt") / f"gt_tum_{name}.txt",
        traj.parent.parent / "datasets" / "gt" / f"gt_tum_{name}.txt",
    )
    return next((path for path in candidates if path.is_file()), None)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot an estimator trajectory.")
    parser.add_argument("output", type=Path, help="output directory or Body_traj.txt")
    parser.add_argument("--gt", type=Path, help="ground-truth trajectory file")
    parser.add_argument("--no-gt", action="store_true", help="do not plot ground truth")
    parser.add_argument("-o", "--save", type=Path, help="save the figure instead of opening a window")
    args = parser.parse_args()

    if args.gt and args.no_gt:
        parser.error("--gt and --no-gt cannot be used together")

    traj_path = output_file(args.output)
    if not traj_path.is_file():
        parser.error(f"trajectory file not found: {traj_path}")

    _, north, east = load_xy(traj_path)
    gt_path = None if args.no_gt else (args.gt or default_ground_truth(traj_path))
    if args.gt and not gt_path.is_file():
        parser.error(f"ground-truth file not found: {gt_path}")

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(east, north, color="tab:blue", linewidth=1.4, label="Output")
    if gt_path is not None:
        _, gt_north, gt_east = load_xy(gt_path, ground_truth=True)
        ax.plot(gt_east, gt_north, color="black", linewidth=1.2, label="Ground truth")
    ax.scatter(east[0], north[0], color="tab:green", s=36, label="Start", zorder=3)
    ax.scatter(east[-1], north[-1], color="tab:red", s=36, label="End", zorder=3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(traj_path.parent.name)
    ax.grid(True, alpha=0.3)
    ax.legend()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=200, bbox_inches="tight")
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
