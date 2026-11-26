"""Summarize the team's GICP node using its per-scan CSV."""

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np

from stand_replay import CSV_FIELDS, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("expected_scans", type=int)
    args = parser.parse_args()
    directory = args.directory
    with (directory / "processing_times.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    poses = [r for r in rows if r["x"]]
    with (directory / "trajectory_gicp.csv").open("w", newline="") as stream:
        out = csv.DictWriter(stream, CSV_FIELDS)
        out.writeheader()
        out.writerows({k: row[k] for k in CSV_FIELDS} for row in poses)
    status = Counter(row["status"] for row in rows)
    values = [float(row["total_ms"]) for row in rows if row["total_ms"]]
    elapsed = (int(poses[-1]["stamp_ns"]) - int(poses[0]["stamp_ns"])) / 1e9
    trajectory = np.array([[float(row[k]) for k in "xyz"] for row in poses])
    delta = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
    bad_run = longest = 0
    for row in rows:
        if row["status"] in ("accepted", "initialized"):
            bad_run = 0
        else:
            bad_run += 1
            longest = max(longest, bad_run)
    metrics = {"expected_scans": args.expected_scans, "received_with_csv_row": len(rows),
               "unaccounted_scans": args.expected_scans - len(rows),
               "scans_with_pose": len(poses), "status": status,
               "total_ms": summary(values), "sync_wait_ms": summary([float(r["wait_ms"]) for r in rows]),
               "registration_ms": summary([float(r["gicp_ms"]) for r in rows if r["gicp_ms"]]),
               "output_rate_hz_from_sensor_stamps": (len(poses) - 1) / elapsed,
               "trajectory_length_m": float(delta.sum()), "max_step_m": float(delta.max()),
               "end_to_start_m": float(np.linalg.norm(trajectory[-1] - trajectory[0])),
               "longest_run_without_accepted_registration": longest}
    (directory / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
