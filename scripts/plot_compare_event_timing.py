#!/usr/bin/env python3
"""
Event vs. Timing Walking Comparison
Compares the torso motion and foot contact forces of an event-based and a timing-based
walking run, from two logs written by biped_walking (DataLogger).

Expected CSV columns: time, swing_leg, imu_accel_{x,y,z}, base_rpy_{roll,pitch,yaw}, f_gt_z_{L,R}
Generates 3 figures (torso IMU acceleration, torso roll / pitch / yaw, ground-truth vertical
contact force of each foot) and prints per-signal statistics over each run's walking phase (first to last swing), or over
--t-start / --t-end when given.

imu_accel is the raw accelerometer: torso frame, gravity included (z ~ +9.81 at rest).
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from plot_utils import REPO_ROOT, setup_style

ACCEL = ("x", "y", "z")
RPY = ("roll", "pitch", "yaw")
FEET = (("L", "Left"), ("R", "Right"))
RUNS = (("event", "Contact event", "#2a78d6"), ("timing", "Timing", "#eb6834"))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare torso acceleration and roll / pitch / yaw of two walking logs."
    )
    parser.add_argument("--event", default="record/contact_event_walking.csv",
                        help="Log of the event-based run (default: record/contact_event_walking.csv)")
    parser.add_argument("--timing", default="record/timing_walking.csv",
                        help="Log of the timing-based run (default: record/timing_walking.csv)")
    parser.add_argument("-s", "--save", default="record/compare_event_timing",
                        help="Path prefix for the saved images; '_<plot>.png' is appended "
                        "(default: record/compare_event_timing). Pass '' to skip saving.")
    parser.add_argument("--t-start", type=float, default=None,
                        help="Start of the plots and statistics [s] (default: first swing)")
    parser.add_argument("--t-end", type=float, default=None,
                        help="End of the plots and statistics [s] (default: last swing)")
    parser.add_argument("--accel-ylim", type=float, default=None,
                        help="Half-range of the acceleration axes around each signal's mean "
                        "[m/s^2], to see the motion between the impacts (default: full range)")
    parser.add_argument("--no-show", action="store_true", help="Do not open the figure windows")
    return parser.parse_args()


def load_run(path, args):
    """Log rows inside the window of this run."""
    path = path if os.path.isabs(path) else os.path.join(REPO_ROOT, path)
    if not os.path.isfile(path):
        sys.exit(f"[Error] Log file not found: {path}")
    print(f"[Plotter] Loading log data from: {os.path.relpath(path, REPO_ROOT)}")
    df = pd.read_csv(path)

    needed = ["time", "swing_leg"] + [f"imu_accel_{a}" for a in ACCEL] + [f"base_rpy_{a}" for a in RPY]
    needed += [f"f_gt_z_{s}" for s, _ in FEET]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        sys.exit(f"[Error] {path} has no column {', '.join(missing)}: log it again with the current biped_walking")

    swing_time = df.loc[df["swing_leg"] != 0, "time"]
    if swing_time.empty and (args.t_start is None or args.t_end is None):
        sys.exit(f"[Error] {path} has no swing phase: give both --t-start and --t-end")
    t0 = swing_time.min() if args.t_start is None else args.t_start
    t1 = swing_time.max() if args.t_end is None else args.t_end
    df = df[(df["time"] >= t0) & (df["time"] <= t1)]
    if df.empty:
        sys.exit(f"[Error] {path} has no data in [{t0}, {t1}] s")
    return df


def plot_signals(runs, columns, labels, title, scale=1.0, ylim=None):
    fig, axes = plt.subplots(len(columns), 1, figsize=(11, 7.5), sharex=True)
    for ax, col, label in zip(axes, columns, labels):
        for key, name, color in RUNS:
            ax.plot(runs[key]["time"], runs[key][col] * scale, color=color, lw=1.0, label=name)
        ax.set_ylabel(label)
        if ylim is not None:
            center = np.mean([runs[key][col].mean() for key, _, _ in RUNS]) * scale
            ax.set_ylim(center - ylim, center + ylim)
    axes[0].set_title(title, loc="left")
    axes[0].legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=len(RUNS), frameon=False, borderaxespad=0.1)
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    return fig


def print_stats(runs, columns, labels, title, scale=1.0):
    """Mean, spread about the mean, largest deviation from the mean and largest magnitude of each signal."""
    print(f"\n{title}")
    print(f"  {'Signal':<10} | {'Run':<13} | {'Mean':>8} | {'Std':>8} | {'Max dev':>8} | {'Max abs':>8}")
    for col, label in zip(columns, labels):
        for key, name, _ in RUNS:
            x = runs[key][col].to_numpy() * scale
            print(f"  {label:<10} | {name:<13} | {x.mean():8.3f} | {x.std():8.3f} | "
                  f"{np.abs(x - x.mean()).max():8.3f} | {np.abs(x).max():8.3f}")


def main():
    args = parse_args()
    setup_style(12)
    runs = {"event": load_run(args.event, args), "timing": load_run(args.timing, args)}
    for key, name, _ in RUNS:
        t = runs[key]["time"]
        print(f"[Plotter] {name}: {t.iloc[0]:.3f} to {t.iloc[-1]:.3f} s")

    accel_cols = [f"imu_accel_{a}" for a in ACCEL]
    rpy_cols = [f"base_rpy_{a}" for a in RPY]
    force_cols = [f"f_gt_z_{s}" for s, _ in FEET]
    deg = 180.0 / np.pi
    print_stats(runs, accel_cols, list(ACCEL), "Torso IMU acceleration [m/s^2]")
    print_stats(runs, rpy_cols, list(RPY), "Torso orientation [deg]", deg)
    print_stats(runs, force_cols, [name for _, name in FEET], "Contact force, ground truth [N]")

    figures = {
        "accel": plot_signals(runs, accel_cols, [f"$a_{a}$ [m/s$^2$]" for a in ACCEL],
                              "Torso IMU acceleration (torso frame, gravity included)", ylim=args.accel_ylim),
        "rpy": plot_signals(runs, rpy_cols, [f"{a.capitalize()} [deg]" for a in RPY], "Torso orientation", deg),
        "contact_force": plot_signals(runs, force_cols, [f"{name} $F_z$ [N]" for _, name in FEET],
                                      "Foot contact force (ground truth, vertical)"),
    }

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
