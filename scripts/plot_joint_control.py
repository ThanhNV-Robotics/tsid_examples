#!/usr/bin/env python3
"""
UR10e Joint Space Control Plotter
Visualizes commanded vs measured joint positions and tracking errors from TSID simulation.
"""

import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot UR10e TSID joint tracking data from CSV log."
    )
    parser.add_argument(
        "-f",
        "--file",
        type=str,
        default="record/ur10_joint_space_control.csv",
        help="Path to the CSV log file (default: record/ur10_joint_space_control.csv)",
    )
    parser.add_argument(
        "-s",
        "--save",
        type=str,
        default="record/ur10_joint_tracking.png",
        help="Path to save the generated plot image (default: record/ur10_joint_tracking.png)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display interactive matplotlib window (useful for headless / batch mode)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Find file
    csv_path = args.file
    if not os.path.exists(csv_path):
        # Check relative to repo root
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        alt_path = os.path.join(repo_root, csv_path)
        if os.path.exists(alt_path):
            csv_path = alt_path
        else:
            print(f"[Error] Log file not found at: {args.file} or {alt_path}")
            sys.exit(1)

    print(f"[Plotter] Loading log data from: {csv_path}")
    df = pd.read_csv(csv_path)

    if "time" not in df.columns:
        print("[Error] CSV does not contain a 'time' column.")
        sys.exit(1)

    time = df["time"].to_numpy()

    # Automatically detect joint names
    cmd_cols = [c for c in df.columns if c.endswith("_pos_cmd")]
    if not cmd_cols:
        print("[Error] No '*_pos_cmd' columns found in CSV.")
        sys.exit(1)

    joint_names = [c[: -len("_pos_cmd")] for c in cmd_cols]
    num_joints = len(joint_names)
    print(f"[Plotter] Detected {num_joints} joints: {', '.join(joint_names)}")

    # Summary table
    print("\n" + "=" * 70)
    print(f"{'Joint Name':<28} | {'RMSE (deg)':<12} | {'Max Err (deg)':<14} | {'RMSE (rad)':<10}")
    print("-" * 70)

    stats = {}
    for name in joint_names:
        cmd_col = f"{name}_pos_cmd"
        meas_col = f"{name}_pos_measured"

        if meas_col not in df.columns:
            print(f"[Warn] Missing measured column for {name}: {meas_col}")
            continue

        q_cmd = df[cmd_col].to_numpy()
        q_meas = df[meas_col].to_numpy()
        err_rad = q_meas - q_cmd
        err_deg = np.rad2deg(err_rad)

        rmse_rad = np.sqrt(np.mean(err_rad**2))
        max_err_deg = np.max(np.abs(err_deg))
        rmse_deg = np.rad2deg(rmse_rad)

        stats[name] = {
            "rmse_deg": rmse_deg,
            "max_err_deg": max_err_deg,
            "rmse_rad": rmse_rad,
            "q_cmd": q_cmd,
            "q_meas": q_meas,
            "err_deg": err_deg,
        }

        print(f"{name:<28} | {rmse_deg:<12.4f} | {max_err_deg:<14.4f} | {rmse_rad:<10.6f}")

    print("=" * 70 + "\n")

    # Determine grid layout
    cols = 2 if num_joints > 1 else 1
    rows = (num_joints + cols - 1) // cols

    # Figure 1: Commanded vs Measured Joint Positions
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3.2 * rows), sharex=True)
    if num_joints == 1:
        axes = np.array([axes])
    axes = axes.flatten()

    fig.suptitle("UR10e TSID Joint Space Tracking ($q_{\\text{cmd}}$ vs $q_{\\text{meas}}$)", fontsize=14, fontweight="bold")

    for i, name in enumerate(joint_names):
        ax = axes[i]
        q_cmd = stats[name]["q_cmd"]
        q_meas = stats[name]["q_meas"]
        rmse_deg = stats[name]["rmse_deg"]

        ax.plot(time, q_cmd, "r--", linewidth=1.8, label="$q_{\\mathrm{cmd}}$ (ref)")
        ax.plot(time, q_meas, "b-", linewidth=1.2, alpha=0.85, label="$q_{\\mathrm{meas}}$ (actual)")

        ax.set_title(f"{name} (RMSE: {rmse_deg:.2f}°)", fontsize=11, fontweight="semibold")
        ax.set_ylabel("Angle [rad]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="upper right", framealpha=0.8, fontsize=9)

    # Label bottom axes
    for c in range(cols):
        idx = (rows - 1) * cols + c
        if idx < len(axes):
            axes[idx].set_xlabel("Time [s]", fontsize=10)
        elif idx - cols < len(axes):
            axes[idx - cols].set_xlabel("Time [s]", fontsize=10)

    # Hide unused subplots
    for j in range(num_joints, len(axes)):
        fig.delaxes(axes[j])

    plt.tight_layout()

    # Save figure
    if args.save:
        save_path = args.save
        if not os.path.isabs(save_path):
            repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            save_path = os.path.join(repo_root, save_path)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        fig.savefig(save_path, dpi=200)
        print(f"[Plotter] Figure saved to: {save_path}")

    # Figure 2: Tracking Errors
    fig_err, axes_err = plt.subplots(rows, cols, figsize=(14, 2.8 * rows), sharex=True)
    if num_joints == 1:
        axes_err = np.array([axes_err])
    axes_err = axes_err.flatten()

    fig_err.suptitle("UR10e Joint Tracking Errors ($e = q_{\\text{meas}} - q_{\\text{cmd}}$)", fontsize=14, fontweight="bold")

    for i, name in enumerate(joint_names):
        ax = axes_err[i]
        err_deg = stats[name]["err_deg"]
        max_deg = stats[name]["max_err_deg"]

        ax.plot(time, err_deg, color="purple", linewidth=1.2, label="Error")
        ax.axhline(0.0, color="black", linestyle=":", linewidth=0.8, alpha=0.7)

        ax.set_title(f"{name} (Max: {max_deg:.2f}°)", fontsize=11, fontweight="semibold")
        ax.set_ylabel("Error [deg]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="upper right", framealpha=0.8, fontsize=9)

    for c in range(cols):
        idx = (rows - 1) * cols + c
        if idx < len(axes_err):
            axes_err[idx].set_xlabel("Time [s]", fontsize=10)
        elif idx - cols < len(axes_err):
            axes_err[idx - cols].set_xlabel("Time [s]", fontsize=10)

    for j in range(num_joints, len(axes_err)):
        fig_err.delaxes(axes_err[j])

    plt.tight_layout()

    err_save_path = None
    if args.save:
        err_save_path = os.path.splitext(save_path)[0] + "_error.png"
        fig_err.savefig(err_save_path, dpi=200)
        print(f"[Plotter] Error plot saved to: {err_save_path}")

    if not args.no_show:
        print("[Plotter] Showing interactive plots...")
        plt.show()


if __name__ == "__main__":
    main()
