#!/usr/bin/env python3
"""
Visualize PGO optimization: before vs after trajectory.

- Before PGO: keyframes.txt (优化前关键帧位姿)
- After PGO:  keyframe_opt.txt (优化后关键帧位姿)
- Optionally draw loop closure constraints between keyframe pairs.

Data directory should contain:
  keyframes.txt     - keyframe poses before PGO (required)
  keyframe_opt.txt  - keyframe poses after PGO (required)
  poses.txt         - TUM full trajectory (optional, for --full-odom)
  loop_constraints.txt - optional: target_id source_id per line

Usage:
  python scripts/visualize_pgo_before_after.py /home/invs/projects/livo2_ws/siat-f10-lc
  python scripts/visualize_pgo_before_after.py /path/to/output_dir --save fig.png --no-show
"""

import argparse
import numpy as np
from pathlib import Path


def load_poses_tum(path):
    """Load TUM format: timestamp tx ty tz qx qy qz qw. Returns list of (t, xyz, quat)."""
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            xyz = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            quat = np.array([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])
            rows.append((t, xyz, quat))
    return rows


def load_keyframes(path):
    """Load keyframes.txt / keyframe_opt.txt: id timestamp tx ty tz qx qy qz qw. Returns list of (id, t, xyz, quat)."""
    keyframes = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 9:
                continue
            kf_id = int(parts[0])
            t = float(parts[1])
            xyz = np.array([float(parts[2]), float(parts[3]), float(parts[4])])
            quat = np.array([float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])])
            keyframes.append((kf_id, t, xyz, quat))
    return keyframes


def get_before_pgo_trajectory(keyframes, poses_tum):
    """
    For each keyframe (id, timestamp), get the pose from full trajectory at nearest timestamp.
    That gives the "before PGO" keyframe trajectory (odometry at keyframe times).
    """
    if not poses_tum:
        return []
    ts = np.array([p[0] for p in poses_tum])
    before = []
    for kf_id, t_kf, _, _ in keyframes:
        idx = np.argmin(np.abs(ts - t_kf))
        _, xyz, quat = poses_tum[idx]
        before.append((kf_id, xyz, quat))
    return before


def load_loop_constraints(path):
    """Load loop_constraints.txt: target_id source_id per line. Returns list of (target_id, source_id)."""
    pairs = []
    if not path.exists():
        return pairs
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                pairs.append((int(parts[0]), int(parts[1])))
    return pairs


def main():
    parser = argparse.ArgumentParser(description="Visualize PGO before/after trajectory")
    parser.add_argument(
        "data_dir",
        type=str,
        default="/home/invs/projects/livo2_ws/siat-f10-lc",
        nargs="?",
        help="Directory containing keyframes.txt, keyframe_opt.txt, (optional) loop_constraints.txt",
    )
    parser.add_argument("--save", type=str, default=None, help="Save figure to this path")
    parser.add_argument("--no-show", action="store_true", help="Do not show interactive window")
    parser.add_argument("--no-loop", action="store_true", help="Do not draw loop closure lines")
    parser.add_argument("--full-odom", action="store_true", help="Also plot full odometry trajectory (poses.txt)")
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    poses_file = data_dir / "poses.txt"
    keyframes_file = data_dir / "keyframes.txt"       # before PGO
    keyframe_opt_file = data_dir / "keyframe_opt.txt" # after PGO
    loop_file = data_dir / "loop_constraints.txt"

    if not keyframes_file.exists():
        print(f"Error: {keyframes_file} not found (before PGO)")
        return 1
    if not keyframe_opt_file.exists():
        print(f"Error: {keyframe_opt_file} not found (after PGO)")
        return 1

    print(f"Data directory: {data_dir.absolute()}")
    print("(Use the same directory as run_livo2_offline output_dir, e.g. ./siat-f10-lc not siat-f10-lc-new)")

    keyframes_before = load_keyframes(keyframes_file)
    keyframes = load_keyframes(keyframe_opt_file)  # after PGO
    kf_ids = [kf[0] for kf in keyframes]

    # Before PGO: from keyframes.txt (same order as after)
    before_by_id = {b[0]: (b[2], b[3]) for b in keyframes_before}
    before_traj = []
    for i, kid in enumerate(kf_ids):
        if kid in before_by_id:
            xyz, quat = before_by_id[kid]
            before_traj.append((kid, xyz, quat))
        else:
            before_traj.append((kid, keyframes[i][2], keyframes[i][3]))
    before_source = "keyframes.txt"
    poses_tum = load_poses_tum(poses_file) if poses_file.exists() else []

    loop_pairs = [] if args.no_loop else load_loop_constraints(loop_file)

    # Build arrays for plotting
    # After PGO: from keyframe_opt.txt (same order as keyframes)
    after_xyz = np.array([kf[2] for kf in keyframes])  # (N, 3)
    kf_ids = [kf[0] for kf in keyframes]
    id_to_idx = {kid: i for i, kid in enumerate(kf_ids)}

    # Before PGO: same order as keyframes
    before_xyz = np.array([b[1] for b in before_traj])  # (N, 3)

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Please install matplotlib: pip install matplotlib")
        return 1

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # Optional: full odometry trajectory (X-Y)
    if args.full_odom:
        full_xyz = np.array([p[1] for p in poses_tum])
        ax.plot(
            full_xyz[:, 0],
            full_xyz[:, 1],
            "c-",
            linewidth=0.6,
            alpha=0.5,
            label="Full odometry",
        )

    # Before PGO trajectory
    ax.plot(
        before_xyz[:, 0],
        before_xyz[:, 1],
        "b-",
        linewidth=1.5,
        label="Before PGO",
        alpha=0.9,
    )
    ax.scatter(
        before_xyz[0, 0],
        before_xyz[0, 1],
        c="blue",
        s=40,
        marker="o",
    )

    # After PGO trajectory
    ax.plot(
        after_xyz[:, 0],
        after_xyz[:, 1],
        "g-",
        linewidth=1.5,
        label="After PGO (optimized)",
        alpha=0.9,
    )
    ax.scatter(
        after_xyz[0, 0],
        after_xyz[0, 1],
        c="green",
        s=40,
        marker="o",
    )

    # Loop closure constraints: line between (target_id, source_id) keyframe positions (after PGO)
    for target_id, source_id in loop_pairs:
        if target_id in id_to_idx and source_id in id_to_idx:
            i_t = id_to_idx[target_id]
            i_s = id_to_idx[source_id]
            pts = np.array([after_xyz[i_t], after_xyz[i_s]])
            ax.plot(
                pts[:, 0],
                pts[:, 1],
                "r--",
                linewidth=1.2,
                alpha=0.8,
            )
    if loop_pairs:
        ax.plot([], [], "r--", linewidth=1.2, label="Loop closure")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("PGO: Before vs After (keyframe trajectory, X-Y)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved: {args.save}")

    if not args.no_show:
        plt.show()
    else:
        plt.close()

    # Print simple stats (higher precision to detect tiny differences)
    diff = np.linalg.norm(after_xyz - before_xyz, axis=1)
    mean_mm = np.mean(diff) * 1000
    max_mm = np.max(diff) * 1000
    print(f"Keyframes: {len(keyframes)}")
    print(f"Before PGO source: {before_source}")
    print(f"Before PGO vs After PGO (position change):")
    print(f"  Mean: {mean_mm:.6f} mm")
    print(f"  Max:  {max_mm:.6f} mm")
    if loop_pairs:
        print(f"Loop constraints: {len(loop_pairs)}")
    if mean_mm < 0.01 and max_mm < 0.01 and loop_pairs:
        print("\n[Warning] Position change is ~0 but loop constraints exist. Possible causes:")
        print("  1) Wrong directory: you must use the SAME output_dir as run_livo2_offline.")
        print("     Example: if you ran with output ./siat-f10-lc, use: python ... visualize_pgo_before_after.py ./siat-f10-lc")
        print("     (Not siat-f10-lc-new or another folder with old/empty data.)")
        print("  2) Old run: keyframes.txt and keyframe_opt.txt were both 'after PGO'. Re-run offline pipeline.")
        print("  3) Check: diff keyframes.txt keyframe_opt.txt  to see if files are identical.")

    return 0


if __name__ == "__main__":
    exit(main())
