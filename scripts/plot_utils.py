import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.special import erf

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DST, LST, RST = "DSt", "LSt", "RSt"  # double support / left stance / right stance (data_type.h LegState)
COLOR = {"L": "tab:blue", "R": "tab:orange", "com": "black"}


# -----------------------------------------------------------------------------
# Plot helpers
# -----------------------------------------------------------------------------
def setup_style(font_size):
    plt.rcParams.update({
        "font.family": "serif",
        "mathtext.fontset": "cm",
        "font.size": font_size,
        "axes.labelsize": font_size,
        "axes.titlesize": font_size,
        "legend.fontsize": font_size - 1,
        "xtick.labelsize": font_size - 1,
        "ytick.labelsize": font_size - 1,
        "axes.grid": True,
        "grid.linestyle": ":",
        "grid.alpha": 0.6,
        "lines.linewidth": 1.3,
        "savefig.bbox": "tight",
    })


def spans(t, mask):
    """(t_start, t_end) of every interval where mask is True."""
    edges = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    s, e = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1
    return [(t[i], t[j]) for i, j in zip(s, e)]


def shade_gait(ax, sim, label=False):
    """Light background: grey = double support, blue / orange = left / right stance."""
    t = sim["t"]
    for st, color, name in ((DST, "0.85", "double support"), (LST, COLOR["L"], "left stance"),
                            (RST, COLOR["R"], "right stance")):
        for k, (t0, t1) in enumerate(spans(t, sim["stance"] == st)):
            ax.axvspan(t0, t1, color=color, alpha=0.10 if st != DST else 0.35, lw=0,
                       label=name if (label and k == 0) else None)


def gait_bars(ax, sim, actual=False):
    """Gait diagram: a bar per leg where it is in (scheduled or actual) contact."""
    t = sim["t"]
    for y, leg in ((1, "L"), (0, "R")):
        sched = sim[f"s_{leg}"] == 1
        for t0, t1 in spans(t, sched):
            ax.barh(y, t1 - t0, left=t0, height=0.5, color=COLOR[leg], alpha=0.35 if actual else 0.9, lw=0)
        if actual:
            for t0, t1 in spans(t, sim[f"c_{leg}"] == 1):
                ax.barh(y, t1 - t0, left=t0, height=0.22, color=COLOR[leg], lw=0)
    ax.set_yticks([1, 0], ["Left", "Right"])
    ax.set_ylim(-0.5, 1.5)
    ax.grid(False, axis="y")


def step_labels(ax, sim, y, t_min=-np.inf):
    """Write 'init shift' and the step number above each phase segment that is
    visible after t_min (segments cut down to less than 0.4 s are skipped)."""
    t = sim["t"]
    segments = [(t0, t1, "init shift") for t0, t1 in spans(t, (sim["stance"] == DST) & (sim["phi_com"] > 0))]
    for k in range(1, sim["step"].max() + 1):
        seg = t[sim["step"] == k]
        if seg.size:
            segments.append((seg[0], seg[-1], f"step {k}"))
    for t0, t1, text in segments:
        t0 = max(t0, t_min)
        if t1 - t0 >= 0.4:
            ax.text(0.5 * (t0 + t1), y, text, ha="center", va="bottom")
