#!/usr/bin/env python3
"""
Biped Joint Control Plotter
Plots commanded vs measured joint positions and commanded joint torques for the
left and right legs from a TSID simulation log (written by DataLogger).

Expected CSV columns: time, pos_cmd_<joint>, pos_measured_<joint>, tau_cmd_<joint>
Generates 4 figures: left/right leg positions and left/right leg torques.
If the log has f_est_z_{L,R} and f_gt_z_{L,R} (or f_touch_{L,R} in older logs),
a figure compares the contact force estimated by the momentum observer with the
simulator's ground-truth contact force.
If the log has the contact fusion columns (p_phase, p_height, p_force, p_fused,
s_hat, s_gt, sole_z), a figure shows the contact estimation of each foot and the
contact state accuracy and touchdown / liftoff detection delays are printed.
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
        "--contact-level",
        type=float,
        default=20.0,
        help="Reference force above which a foot counts as in contact, for the shading "
        "and the contact/swing RMSE [N] (default: 20, same as Cheat_StateEstimator)",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=3.0,
        help="Start of walking [s] (T_stand); accuracy and delays are computed from here (default: 3)",
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


def contact_spans(time, in_contact):
    """(t_start, t_end) of every interval where in_contact is True."""
    edges = np.diff(np.concatenate(([0], in_contact.astype(int), [0])))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1
    return [(time[s], time[e]) for s, e in zip(starts, ends)]


def plot_contact_forces(df, time, contact_level, ref):
    """Estimated (momentum observer) vs reference vertical force per foot, and their sum.

    ref: column prefix of the reference force, "f_gt_z" (MuJoCo contact forces)
    or "f_ref" (touch sensors, older logs)
    """
    ref_name = "ground truth" if ref == "f_gt_z" else "touch sensor"
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle(f"Foot contact force: observer estimate vs {ref_name}", fontsize=14, fontweight="bold")
    print(f"\nContact force ({ref_name} > {contact_level:.0f} N = in contact)")
    print(f"  {'Foot':<6} | {'RMSE (N)':>8} | {'RMSE contact (N)':>16} | {'RMSE swing (N)':>14} | {'Max err (N)':>11}")
    for ax, side, name in zip(axes, ("L", "R"), ("Left", "Right")):
        f_est = df[f"f_est_z_{side}"].to_numpy()
        f_ref = df[f"{ref}_{side}"].to_numpy()
        err = f_est - f_ref
        in_contact = f_ref > contact_level

        def rmse(mask):
            return np.sqrt(np.mean(err[mask] ** 2)) if mask.any() else float("nan")

        rmse_all, rmse_st, rmse_sw = rmse(np.ones_like(in_contact)), rmse(in_contact), rmse(~in_contact)
        print(f"  {name:<6} | {rmse_all:>8.2f} | {rmse_st:>16.2f} | {rmse_sw:>14.2f} | {np.max(np.abs(err)):>11.2f}")

        # Shade the intervals where the reference reports contact
        for k, (t0, t1) in enumerate(contact_spans(time, in_contact)):
            ax.axvspan(t0, t1, color="tab:gray", alpha=0.12, label=f"contact ({ref_name})" if k == 0 else None)
        ax.plot(time, f_ref, "r--", linewidth=1.6, label=f"$f_z$ ({ref_name})")
        ax.plot(time, f_est, "b-", linewidth=1.1, alpha=0.85, label="$\\hat f_z$ (observer)")
        ax.axhline(0.0, color="black", linestyle=":", linewidth=0.8, alpha=0.7)
        ax.set_title(f"{name} foot (RMSE contact {rmse_st:.1f} N, swing {rmse_sw:.1f} N)",
                     fontsize=11, fontweight="semibold")
        ax.set_ylabel("$F_z$ [N]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="upper right", framealpha=0.8, fontsize=9)

    # Sum of both feet: the total vertical ground reaction (about m g when the
    # CoM does not accelerate vertically)
    ax = axes[2]
    est_sum = df["f_est_z_L"].to_numpy() + df["f_est_z_R"].to_numpy()
    ref_sum = df[f"{ref}_L"].to_numpy() + df[f"{ref}_R"].to_numpy()
    ax.plot(time, ref_sum, "r--", linewidth=1.6, label=f"$f_{{z,L}} + f_{{z,R}}$ ({ref_name})")
    ax.plot(time, est_sum, "b-", linewidth=1.1, alpha=0.85, label="$\\hat f_{z,L} + \\hat f_{z,R}$")
    ax.set_title(f"Total vertical force (mean {ref_name} {np.mean(ref_sum):.1f} N, "
                 f"mean estimate {np.mean(est_sum):.1f} N)", fontsize=11, fontweight="semibold")
    ax.set_ylabel("$F_z$ [N]", fontsize=10)
    ax.set_xlabel("Time [s]", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.8, fontsize=9)
    fig.tight_layout()
    return fig


def match_edges(t, ref, est, rising, window=0.3):
    """Delay of each reference edge: time from a rising (or falling) edge of the
    reference contact state to the nearest same-direction edge of the estimate
    within +-window [s]. Positive = estimate late. Returns (delays, n_missed)."""
    d = 1 if rising else -1
    t_ref = t[1:][np.diff(ref.astype(int)) == d]
    t_est = t[1:][np.diff(est.astype(int)) == d]
    delays, missed = [], 0
    for tr in t_ref:
        if t_est.size and np.min(np.abs(t_est - tr)) <= window:
            delays.append(t_est[np.argmin(np.abs(t_est - tr))] - tr)
        else:
            missed += 1
    return np.asarray(delays), missed


def contact_estimation_metrics(df, time, t_start):
    """Print the accuracy of s_hat against s_gt and the touchdown / liftoff delays."""
    walk = time >= t_start
    print(f"\nContact estimation (from t = {t_start:.1f} s)")
    print(f"  {'Foot':<6} | {'Accuracy':>8} | {'Touchdown delay [ms]':>24} | {'Liftoff delay [ms]':>24} | {'Est. switches':>13}")
    for side, name in (("L", "Left"), ("R", "Right")):
        s_hat = df[f"s_hat_{side}"].to_numpy()[walk] > 0.5
        s_gt = df[f"s_gt_{side}"].to_numpy()[walk] > 0.5
        t = time[walk]
        acc = 100.0 * np.mean(s_hat == s_gt)
        cells = []
        for rising in (True, False):
            dl, missed = match_edges(t, s_gt, s_hat, rising)
            cell = (f"mean {1e3 * dl.mean():5.1f}, max {1e3 * np.max(np.abs(dl)):5.1f}" if dl.size else "-")
            cells.append(cell + (f" ({missed} missed)" if missed else ""))
        n_sw = int(np.sum(np.diff(s_hat.astype(int)) != 0))
        n_sw_gt = int(np.sum(np.diff(s_gt.astype(int)) != 0))
        print(f"  {name:<6} | {acc:>7.2f}% | {cells[0]:>24} | {cells[1]:>24} | {n_sw:>5} (gt {n_sw_gt})")


def plot_contact_estimation(df, time, t_start):
    """Per foot: model probabilities and fused probability, estimated vs true
    contact state, and sole height."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 9), sharex=True,
                             gridspec_kw={"height_ratios": [1.4, 0.7, 1]})
    fig.suptitle("Contact estimation (Kalman fusion) vs ground truth", fontsize=14, fontweight="bold")
    for col, (side, name) in enumerate((("L", "Left"), ("R", "Right"))):
        s_gt = df[f"s_gt_{side}"].to_numpy() > 0.5
        shade = contact_spans(time, s_gt)

        # 1. Probabilities: the three models (thin) and the fused estimate (thick)
        ax = axes[0, col]
        for k, (t0, t1) in enumerate(shade):
            ax.axvspan(t0, t1, color="tab:gray", alpha=0.12, label="contact (ground truth)" if k == 0 else None)
        ax.plot(time, df[f"p_phase_{side}"], color="tab:green", lw=0.9, alpha=0.8, label="phase $P(c|s_\\phi,\\phi)$")
        ax.plot(time, df[f"p_height_{side}"], color="tab:purple", lw=0.9, alpha=0.8, label="height $P(c|p_z)$")
        ax.plot(time, df[f"p_force_{side}"], color="tab:orange", lw=0.9, alpha=0.8, label="force $P(c|f_z)$")
        ax.plot(time, df[f"p_fused_{side}"], color="tab:blue", lw=1.8, label="fused $P(c)$")
        ax.set_title(f"{name} foot", fontsize=11, fontweight="semibold")
        ax.set_ylabel("probability", fontsize=10)
        ax.set_ylim(-0.05, 1.05)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="lower left", framealpha=0.85, fontsize=8, ncol=2)

        # 2. Contact state: estimate (s_hat) vs ground truth
        ax = axes[1, col]
        ax.step(time, s_gt.astype(float), "r--", where="post", lw=1.6, label="ground truth")
        ax.step(time, df[f"s_hat_{side}"].to_numpy(), "b-", where="post", lw=1.1, alpha=0.85, label="estimate $\\hat s$")
        ax.set_yticks([0, 1], ["swing", "contact"])
        ax.set_ylim(-0.2, 1.2)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.legend(loc="center left", framealpha=0.85, fontsize=8)

        # 3. Sole height, the input of the height model
        ax = axes[2, col]
        for t0, t1 in shade:
            ax.axvspan(t0, t1, color="tab:gray", alpha=0.12)
        ax.plot(time, 1e3 * df[f"sole_z_{side}"].to_numpy(), color="tab:purple", lw=1.2)
        ax.axhline(0.0, color="black", ls=":", lw=0.8)
        ax.set_ylabel("sole height [mm]", fontsize=10)
        ax.set_xlabel("Time [s]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.6)

    for ax in axes.flat:
        ax.axvline(t_start, color="0.4", lw=0.8)
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
    # A run stopped mid-write can leave an incomplete last line
    df = df[df.iloc[:, -1].notna()].reset_index(drop=True)
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
        # Joint angle plots disabled
        # if leg_pos:
        #     figures[f"{leg}_position"] = plot_positions(df, time, leg, leg_pos)
        if leg_tau:
            figures[f"{leg}_torque"] = plot_torques(df, time, leg, leg_tau)
    if not figures:
        print("[Error] No joints named 'left_*' or 'right_*' found in CSV.")
        sys.exit(1)

    # Reference force: MuJoCo contact forces, or the touch sensors in older logs
    ref = next((r for r in ("f_gt_z", "f_touch") if f"{r}_L" in df.columns and f"{r}_R" in df.columns), None)
    if ref and "f_est_z_L" in df.columns and "f_est_z_R" in df.columns:
        figures["contact_force"] = plot_contact_forces(df, time, args.contact_level, ref)
    else:
        print("\n[Plotter] No contact force columns (f_est_z_*, f_gt_z_*), skipping contact force plot")

    fusion_cols = [f"{c}_{s}" for c in ("p_phase", "p_height", "p_force", "p_fused", "s_hat", "s_gt", "sole_z")
                   for s in ("L", "R")]
    if all(c in df.columns for c in fusion_cols):
        contact_estimation_metrics(df, time, args.t_start)
        figures["contact_estimation"] = plot_contact_estimation(df, time, args.t_start)
    else:
        print("\n[Plotter] No contact fusion columns (p_fused_*, s_hat_*, ...), skipping contact estimation plot")

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
