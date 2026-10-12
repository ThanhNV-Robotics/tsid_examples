import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.special import erf

# Same plot style, colours and interval helper as the report figures
from plot_utils import COLOR, setup_style, spans

SAVED_PATH = "/home/vanthanhnguyen/Documents/PhD_Note/figures"
figure_name = "Gait Phase Planner Diagram.png"

contact_schedule = [True, False] # assume left leg swing first

phi_L, phi_R = 0.0, 0.0 # phase variable for left and right legs
T = 1 # swing time/period
dt = 0.001 # sampling time

num_steps = 4 # number of step
time = []
phi_L_data, phi_R_data = [], []
swing_data = [] # leg swinging at each tick: "L" or "R"
step_count = 0
sim_time = 0

# Phase planning
while (step_count < num_steps):
    if (contact_schedule[0]):
        phi_L += dt/T # left swing
        phi_R = 0.0
        if (phi_L >= 1): #  end of swing 
            phi_L = 0.0
            contact_schedule[0] = False
            contact_schedule[1] = True
            step_count += 1
    if (contact_schedule[1]):
        phi_L = 0.0
        phi_R += dt/T # Right swing
        if (phi_R >= 1): # end of swing
            phi_R = 0.0
            contact_schedule[0] = True
            contact_schedule[1] = False
            step_count += 1
    sim_time += dt
    time.append(sim_time)
    phi_L_data.append(phi_L)
    phi_R_data.append(phi_R)
    swing_data.append("L" if contact_schedule[0] else "R")

time_p = time[1000:2000]
# muy_contact


#------------------------------------------------------------------
# Plotting Stuffs
#------------------------------------------------------------------
print ("Done")
time = np.asarray(time)
swing_data = np.asarray(swing_data)

setup_style(9)
fig, axes = plt.subplots(2, 1, figsize=(6.0, 3.6), sharex=True,
                         gridspec_kw={"height_ratios": [1, 0.55]})

# Foot phases, background coloured by the stance leg (the leg that does not swing)
ax = axes[0]
for leg, name in (("R", "left stance"), ("L", "right stance")):
    stance_color = COLOR["L" if leg == "R" else "R"]
    for k, (t0, t1) in enumerate(spans(time, swing_data == leg)):
        ax.axvspan(t0, t1, color=stance_color, alpha=0.10, lw=0, label=name if k == 0 else None)
ax.plot(time, phi_L_data, color=COLOR["L"], label=r"$\phi_L$ (left foot)")
ax.plot(time, phi_R_data, color=COLOR["R"], ls="--", label=r"$\phi_R$ (right foot)")
# Swing leg above each step (skips the 1-tick segment left after the last step)
for leg in ("L", "R"):
    for t0, t1 in spans(time, swing_data == leg):
        if t1 - t0 >= 0.5 * T:
            ax.text(0.5 * (t0 + t1), 1.03, f"{leg} swing", ha="center", va="bottom")
ax.set_ylabel("foot phase")
ax.set_ylim(-0.05, 1.22)

# Gait diagram: wide bar = stance, thin line = swing
ax = axes[1]
for y, leg in ((1, "L"), (0, "R")):
    for t0, t1 in spans(time, swing_data != leg):
        ax.barh(y, t1 - t0, left=t0, height=0.5, color=COLOR[leg], lw=0)
    for t0, t1 in spans(time, swing_data == leg):
        ax.barh(y, t1 - t0, left=t0, height=0.08, color=COLOR[leg], alpha=0.6, lw=0)
ax.set_yticks([1, 0], ["Left", "Right"])
ax.set_ylim(-0.5, 1.5)
ax.grid(False, axis="y")
ax.set_xlabel("time [s]")
ax.set_title("contact schedule (bar = stance, line = swing)", loc="left", pad=2)

axes[0].set_xlim(time[0], time[-1])
fig.align_ylabels(axes)
fig.tight_layout(h_pad=0.4, rect=(0, 0, 1, 0.9))
fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center", ncol=4, frameon=False)

os.makedirs(SAVED_PATH, exist_ok=True)
save_file = os.path.join(SAVED_PATH, figure_name)
fig.savefig(save_file, dpi=300)
print(f"Saved {save_file}")
plt.show()
        

            

