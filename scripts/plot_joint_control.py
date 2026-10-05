#!/usr/bin/env python3
"""
Biped Joint Control Plotter
Plots commanded vs measured joint positions and commanded joint torques for the
left and right legs from a TSID simulation log (written by DataLogger).

Expected CSV columns: time, pos_cmd_<joint>, pos_measured_<joint>, tau_cmd_<joint>
Generates 4 figures: left/right leg positions and left/right leg torques.
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEGS = ("left", "right")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot biped TSID joint positions and torques from a CSV log."
    )
    parser.add_argument(
        "-f",
        "--file",
        type=str,
        default="record/biped_standing_joint_control.csv",
        help="Path to the CSV log file (default: record/biped_standing_joint_control.csv)",
    )
    parser.add_argument(
        "-s",
        "--save",
        type=str,
        default="record/biped_joint_control",
        help="Path prefix for the saved images; '_<plot>.png' is appended "
        "(default: record/biped_joint_control). Pass '' to skip saving.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display interactive matplotlib windows (useful for headless / batch mode)",
    )
    return parser.parse_args()


def resolve_path(path):
    """Return path as given if it exists or is absolute, otherwise relative to the repo root."""
    if os.path.isabs(path) or os.path.exists(path):
        return path
    return os.path.join(REPO_ROOT, path)


def joints_with_prefix(columns, prefix):
    """Joint names of all columns named '<prefix>_<joint>', in file order."""
    tag = prefix + "_"
    return [c[len(tag):] for c in columns if c.startswith(tag)]


def short_name(joint, leg):
    """'left_knee_pitch_joint' -> 'knee_pitch'"""
    name = joint[len(leg) + 1:] if joint.startswith(leg + "_") else joint
    return name[: -len("_joint")] if name.endswith("_joint") else name


def make_grid(n, title):
    cols = 2 if n > 1 else 1
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(14, 3.0 * rows), sharex=True, squeeze=False)
    fig.suptitle(title, fontsize=14, fontweight="bold")
    axes = axes.flatten()
    for ax in axes[n:]:
        fig.delaxes(ax)
    for ax in axes[:n][-cols:]:
        ax.set_xlabel("Time [s]", fontsize=10)
    return fig, axes[:n]


def plot_positions(df, time, leg, joints):
    fig, axes = make_grid(len(joints), f"{leg.capitalize()} leg joint positions "
                          "($q_{\\mathrm{cmd}}$ vs $q_{\\mathrm{meas}}$)")
    print(f"\n{leg.capitalize()} leg tracking")
    print(f"  {'Joint':<16} | {'RMSE (deg)':>10} | {'Max err (deg)':>13}")
    for ax, joint in zip(axes, joints):
        q_cmd = df[f"pos_cmd_{joint}"].to_numpy()
        q_meas = df[f"pos_measured_{joint}"].to_numpy()
        err_deg = np.rad2deg(q_meas - q_cmd)
        rmse_deg = np.sqrt(np.mean(err_deg**2))
        max_err_deg = np.max(np.abs(err_deg))
        print(f"  {short_name(joint, leg):<16} | {rmse_deg:>10.3f} | {max_err_deg:>13.3f}")

        ax.plot(time, q_cmd, "r--", linewidth=1.8, label="$q_{\\mathrm{cmd}}$")
        ax.plot(time, q_meas, "b-", linewidth=1.2, alpha=0.85, label="$q_{\\mathrm{meas}}$")
        ax.set_title(f"{short_name(joint, leg)} (RMSE: {rmse_deg:.2f}°)", fontsize=11, fontweight="semibold")
        ax.set_ylabel("Angle [rad]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="best", framealpha=0.8, fontsize=9)
    fig.tight_layout()
    return fig


def plot_torques(df, time, leg, joints):
    fig, axes = make_grid(len(joints), f"{leg.capitalize()} leg joint torques ($\\tau_{{\\mathrm{{cmd}}}}$)")
    for ax, joint in zip(axes, joints):
        tau = df[f"tau_cmd_{joint}"].to_numpy()
        ax.plot(time, tau, color="tab:green", linewidth=1.2)
        ax.axhline(0.0, color="black", linestyle=":", linewidth=0.8, alpha=0.7)
        ax.set_title(f"{short_name(joint, leg)} (peak: {np.max(np.abs(tau)):.1f} Nm)",
                     fontsize=11, fontweight="semibold")
        ax.set_ylabel("Torque [Nm]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.6)
    fig.tight_layout()
    return fig


def main():
    args = parse_args()

    csv_path = resolve_path(args.file)
    if not os.path.exists(csv_path):
        print(f"[Error] Log file not found: {args.file}")
        sys.exit(1)

    print(f"[Plotter] Loading log data from: {csv_path}")
    df = pd.read_csv(csv_path)
    if "time" not in df.columns:
        print("[Error] CSV does not contain a 'time' column.")
        sys.exit(1)
    time = df["time"].to_numpy()

    pos_joints = joints_with_prefix(df.columns, "pos_cmd")
    tau_joints = joints_with_prefix(df.columns, "tau_cmd")
    if not pos_joints:
        print("[Error] No 'pos_cmd_<joint>' columns found in CSV.")
        sys.exit(1)
    missing = [j for j in pos_joints if f"pos_measured_{j}" not in df.columns]
    if missing:
        print(f"[Error] Missing 'pos_measured_<joint>' columns for: {', '.join(missing)}")
        sys.exit(1)

    figures = {}
    for leg in LEGS:
        leg_pos = [j for j in pos_joints if j.startswith(leg + "_")]
        leg_tau = [j for j in tau_joints if j.startswith(leg + "_")]
        if leg_pos:
            figures[f"{leg}_position"] = plot_positions(df, time, leg, leg_pos)
        if leg_tau:
            figures[f"{leg}_torque"] = plot_torques(df, time, leg, leg_tau)
    if not figures:
        print("[Error] No joints named 'left_*' or 'right_*' found in CSV.")
        sys.exit(1)

    if args.save:
        prefix = args.save if os.path.isabs(args.save) else os.path.join(REPO_ROOT, args.save)
        os.makedirs(os.path.dirname(prefix) or ".", exist_ok=True)
        print()
        for name, fig in figures.items():
            path = f"{prefix}_{name}.png"
            fig.savefig(path, dpi=150)
            print(f"[Plotter] Saved {path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
