"""Render map, trajectory and scan timings using the saved results."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory
    with (directory / "map_kiss.pcd").open("rb") as stream:
        for line in stream:
            if line == b"DATA binary\n":
                break
        else:
            raise ValueError("Expected a binary XYZ PCD")
        points = np.fromfile(stream, dtype="<f4").reshape(-1, 3)
    traj = np.atleast_1d(np.genfromtxt(directory / "trajectory_kiss.csv", delimiter=",", names=True))
    poses = np.column_stack([traj[axis] for axis in "xyz"])
    times = np.atleast_1d(np.genfromtxt(directory / "processing_times.csv", delimiter=",", names=True))
    plt.style.use("dark_background")
    for filename, is_3d in (("map_top.png", False), ("map_perspective.png", True)):
        fig = plt.figure(figsize=(11, 7), layout="constrained")
        ax = fig.add_subplot(projection="3d" if is_3d else None)
        coords = points.T if is_3d else points.T[:2]
        colors = ax.scatter(*coords, c=points[:, 2], cmap="viridis", s=0.55, alpha=0.65,
                            rasterized=True)
        ax.plot(*(poses.T if is_3d else poses.T[:2]), color="#ff8b63", lw=1.5, label="KISS-ICP trajectory")
        ax.scatter(*(poses[0] if is_3d else poses[0, :2]), color="white", s=22, label="Start")
        ax.legend(loc="upper right")
        ax.set(xlabel="X (m)", ylabel="Y (m)", title="Unitree Go2 • KISS-ICP map")
        if is_3d:
            ax.set_zlabel("Z (m)")
            ax.set_box_aspect(np.maximum(np.ptp(points, axis=0), .01))
            ax.view_init(elev=40, azim=-55)
        else:
            ax.set_aspect("equal")
        fig.colorbar(colors, ax=ax, label="Height Z (m)", shrink=0.74)
        fig.savefig(directory / filename, dpi=160)
        plt.close(fig)

    fig, (xy, z) = plt.subplots(1, 2, figsize=(11, 5.4), layout="constrained")
    elapsed = (traj["stamp_ns"] - traj["stamp_ns"][0]) / 1e9
    line = xy.scatter(poses[:, 0], poses[:, 1], c=elapsed, s=5, cmap="plasma")
    xy.set(xlabel="X (m)", ylabel="Y (m)", aspect="equal", title="Horizontal trajectory")
    fig.colorbar(line, ax=xy, label="Elapsed time (s)")
    z.plot(elapsed, poses[:, 2], lw=1)
    z.set(xlabel="Elapsed time (s)", ylabel="Z (m)", title="Vertical trajectory")
    fig.savefig(directory / "trajectory.png", dpi=160)
    plt.close(fig)

    fig, (latency, distribution) = plt.subplots(1, 2, figsize=(11, 5.4), layout="constrained")
    secs = (times["record_ns"] - times["record_ns"][0]) / 1e9
    for key, label in (("node_callback_ms", "KISS-ICP callback"), ("output_lag_ms", "Lag to map append")):
        latency.plot(secs, times[key], lw=0.6, alpha=0.8, label=label)
        sorted_values = np.sort(times[key][np.isfinite(times[key])])
        distribution.plot(sorted_values, np.arange(1, len(sorted_values) + 1) / len(sorted_values),
                          label=label)
    latency.axhline(71, color="#ff8b63", ls="--", lw=0.9, label="71 ms budget")
    latency.set(xlabel="Bag elapsed time (s)", ylabel="Time (ms)", title="Per-scan timing")
    distribution.set(xlabel="Time (ms)", ylabel="Fraction of scans", xscale="symlog",
                     title="Cumulative distribution")
    latency.legend()
    distribution.legend()
    fig.savefig(directory / "processing_times.png", dpi=160)
    plt.close(fig)

    displacement = np.linalg.norm(np.diff(poses, axis=0), axis=1)
    quat = np.column_stack([traj[name] for name in ("qx", "qy", "qz", "qw")])
    angles = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(quat[1:]*quat[:-1], axis=1)), 0, 1)))
    geometry = {"map_min_xyz_m": points.min(axis=0).tolist(),
                "map_max_xyz_m": points.max(axis=0).tolist(),
                "trajectory_length_m": float(displacement.sum()),
                "end_to_start_m": float(np.linalg.norm(poses[-1] - poses[0])),
                "max_step_m": float(displacement.max()), "max_step_deg": float(angles.max()),
                "step_p99_m": float(np.percentile(displacement, 99)),
                "step_p99_deg": float(np.percentile(angles, 99)),
                "max_step_at_s": float(elapsed[np.argmax(displacement) + 1]),
                "max_turn_at_s": float(elapsed[np.argmax(angles) + 1])}
    (directory / "geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    print(json.dumps(geometry, indent=2))


if __name__ == "__main__":
    main()
