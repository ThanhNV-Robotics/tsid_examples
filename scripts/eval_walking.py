#!/usr/bin/env python3
"""
Headless walking evaluation for biped_walking (doc/event_based_control.md, section 3).

Runs the demo without windows for a given simulated time (or only analyses an
existing log), then summarises how every step ended and how stable the walk was:

  - survival time, QP failure
  - step endings: early touchdown event / late -> contact event / late -> timeout
  - max base roll and pitch while walking
  - lateral CoM error at touchdown (planned vs measured CoM)
  - share of walking time with the planned ZMP at the lateral sole limit

Usage:
  python3 scripts/eval_walking.py --run --time 60 --name baseline
  python3 scripts/eval_walking.py --csv record/biped_standing_joint_control.csv --log run.log
"""

import argparse
import os
import shutil
import subprocess
import sys

import numpy as np
import pandas as pd

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = os.path.join(REPO_ROOT, "build", "biped_walking")
DEMO_CSV = os.path.join(REPO_ROOT, "record", "biped_standing_joint_control.csv")


def parse_args():
    p = argparse.ArgumentParser(description="Run biped_walking headless and summarise the walk.")
    p.add_argument("--run", action="store_true", help="run the demo headless first (TSID_HEADLESS=1)")
    p.add_argument("--time", type=float, default=60.0, help="simulated time for --run [s] (default 60)")
    p.add_argument("--name", type=str, default="eval",
                   help="--run: copies the log and CSV to record/eval_<name>.{log,csv}")
    p.add_argument("--csv", type=str, help="analyse this CSV instead of running")
    p.add_argument("--log", type=str, help="console log of the run (for timeouts / QP failure)")
    p.add_argument("--t-start", type=float, default=4.0, help="start of stepping [s] (default 4)")
    p.add_argument("--sole-y-half", type=float, default=0.02,
                   help="lateral ZMP limit of the walk planner [m] (WalkPlanner::sole_y_half_, default 0.02)")
    return p.parse_args()


def run_demo(sim_time, name):
    env = dict(os.environ, TSID_HEADLESS="1", TSID_SIM_TIME=str(sim_time))
    log_path = os.path.join(REPO_ROOT, "record", f"eval_{name}.log")
    print(f"[eval] running {DEMO} for {sim_time:g} s (headless) ...")
    with open(log_path, "w") as log:
        subprocess.run([DEMO], cwd=REPO_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                       timeout=max(600, 20 * sim_time))
    csv_path = os.path.join(REPO_ROOT, "record", f"eval_{name}.csv")
    shutil.copyfile(DEMO_CSV, csv_path)
    return csv_path, log_path


def summarise(csv_path, log_path, t_start, sole_y_half):
    d = pd.read_csv(csv_path)
    d = d[d.iloc[:, -1].notna()].reset_index(drop=True)  # drop an incomplete last line
    log = open(log_path).read() if log_path and os.path.exists(log_path) else ""

    t = d.time.to_numpy()
    leg = d.swing_leg.to_numpy()
    late = d.late_touchdown.to_numpy() if "late_touchdown" in d else np.zeros(len(t))
    phase = d.swing_phase.to_numpy()
    walk = t >= t_start

    # Step endings, from the swing-leg changes
    changes = np.flatnonzero(np.diff(leg) != 0) + 1
    n_early, end_phase, late_ms, td_com_err, width_dev = 0, [], [], [], []
    import re
    m = re.search(r"stance width ([0-9.]+) m", log)
    nominal_width = float(m.group(1)) if m else None
    for a, b in zip(changes[:-1], changes[1:]):
        if leg[a] == 0:
            continue
        n_late = int(late[a:b].sum())
        if n_late:
            late_ms.append(n_late)
        elif phase[b - 1] < 0.999:
            n_early += 1
            end_phase.append(phase[b - 1])
        # one tick before the touchdown: at the touchdown tick the walk planner
        # resets its plan toward the measured state (cp_feedback_gain)
        td_com_err.append(1e3 * (d.com_meas_y[b - 1] - d.plan_com_y[b - 1]))
        # Step width at touchdown vs the nominal width (foothold adaptation)
        if nominal_width is not None:
            width = abs(d.lfoot_meas_y[b] - d.rfoot_meas_y[b])
            width_dev.append(1e3 * (width - nominal_width))
    n_steps = n_early + len(late_ms) + sum(
        1 for a, b in zip(changes[:-1], changes[1:]) if leg[a] != 0 and late[a:b].sum() == 0 and phase[b - 1] >= 0.999)
    n_timeout = log.count("late touchdown timeout")

    # Planned ZMP at the lateral sole limit (relative to the stance ankle, world y;
    # the walk heads along x, so world y is the lateral axis)
    stance_left = leg == -1   # right foot swings -> left stance
    stance_right = leg == 1
    ankle_y = np.where(stance_left, d.lfoot_meas_y, np.where(stance_right, d.rfoot_meas_y, np.nan))
    zmp_rel = np.abs(d.plan_zmp_y.to_numpy() - ankle_y)
    single = walk & (stance_left | stance_right)
    if "double_support" in d:  # the double support has the ZMP between the feet
        single &= d.double_support.to_numpy() < 0.5
    sat = np.mean(zmp_rel[single] >= sole_y_half - 5e-4) if single.any() else float("nan")

    roll = np.degrees(np.abs(d.base_rpy_roll.to_numpy()))
    pitch = np.degrees(np.abs(d.base_rpy_pitch.to_numpy()))

    print(f"\n=== {os.path.basename(csv_path)}")
    print(f"ran to t = {t[-1]:.2f} s | QP failed: {'QP failed' in log} | steps: {n_steps}")
    print(f"max |roll| {roll[walk].max():.2f} deg, max |pitch| {pitch[walk].max():.2f} deg (while walking)")
    print(f"step endings: early event {n_early}, late {len(late_ms)} "
          f"(-> timeout {n_timeout}, -> contact event {len(late_ms) - n_timeout})")
    if end_phase:
        print(f"  early events end at phase: mean {np.mean(end_phase):.3f}, min {np.min(end_phase):.3f}")
    if late_ms:
        print(f"  late duration: mean {np.mean(late_ms):.1f} ms, max {np.max(late_ms)} ms")
    if td_com_err:
        e = np.asarray(td_com_err)
        print(f"lateral CoM error at touchdown (measured - planned): mean |e| {np.mean(np.abs(e)):.1f} mm, "
              f"max |e| {np.max(np.abs(e)):.1f} mm")
    print(f"planned ZMP at the lateral sole limit: {100 * sat:.1f} % of single support")
    if width_dev:
        w = np.asarray(width_dev)
        print(f"step width at touchdown - nominal ({1e3 * nominal_width:.0f} mm): mean {np.mean(w):+.1f} mm, "
              f"range [{np.min(w):+.1f}, {np.max(w):+.1f}] mm")
    acc = [line.strip() for line in log.splitlines() if "accuracy" in line]
    if acc:
        print(acc[-1])


def main():
    args = parse_args()
    if args.run:
        csv_path, log_path = run_demo(args.time, args.name)
    elif args.csv:
        csv_path, log_path = args.csv, args.log
    else:
        sys.exit("give --run or --csv")
    summarise(csv_path, log_path, args.t_start, args.sole_y_half)


if __name__ == "__main__":
    main()
