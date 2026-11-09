# LIP walk planner

This document explains how the walking references are planned, each control cycle (500 Hz), for
the TSID tasks:

- the CoM position, velocity and acceleration (CoM task);
- the swing foot pose, velocity and acceleration (swing task);
- the torso yaw (base task).

It is split over two classes:

| Class | File | Does |
|---|---|---|
| `LIPTaskPlanner` | [lip_task_planner.cpp](../src/planning/lip_task_planner.cpp) | LIP / capture point model, receding-horizon ZMP, CP feedback, CoM outputs |
| `WalkPlanner` | [walk_planner.cpp](../src/planning/walk_planner.cpp) | phases and timing, CP targets, footholds and their adaptation, swing trajectory, heading, support frame |

It comes from the ROS 2 project (Stages 1, 2, 3 and 3b of
[event_based_control.md](event_based_control.md); first version `python/WalkPlanner.py`). The settings are in the `LIPPlanner` section
of `config/tsid_wbc.yaml`.

**Notation**

| Symbol | Meaning |
|---|---|
| $c$ | CoM (x, y) |
| $\xi$ | capture point (CP), also called the divergent component of motion |
| $p$ | ZMP |
| $z_c$ | LIP height |
| $\omega = \sqrt{g / z_c}$ | LIP natural frequency |
| $T$ | swing time `Tswing` |
| $t_{ds}$ | double support time `double_support_time` |
| $\Delta t$ | planner period (1 / `sampling_frequency`) |
| $E_{ss} = e^{\omega T}$, $E_{ds} = e^{\omega t_{ds}}$ | CP growth over single / double support |

All the planar quantities are expressed in the **support frame**: at the stance ankle, rotated by
the stance foot's yaw only, with x forward and z up. The frame is fixed during a step.

---

## 1. LIP / capture point model

The CoM is modelled as a point mass at constant height $z_c$ over the ZMP. The CP
$\xi = c + \dot c / \omega$ splits the LIP into a stable part and an unstable part:

$$
\dot c = \omega\,(\xi - c) \qquad \text{(the CoM follows the CP: stable)}
$$

$$
\dot \xi = \omega\,(\xi - p) \qquad \text{(the CP diverges away from the ZMP: unstable)}
$$

The planner controls the CP through the ZMP. The CoM then follows the CP by itself.

**LIP height.** `reset()` sets $z_c$ to the measured CoM height above the ground in the pose
walking starts from, which is the init-walking pose. The yaml's `com_height` is only the default
before that.

### 1.1 Exact discretization

For a ZMP held constant over one period $\Delta t$, both equations have closed-form solutions, and
the planner integrates with them (`ComputeCP`, `ComputeCoM`):

$$
\xi_{k+1} = p_k + e^{\omega \Delta t}\,(\xi_k - p_k)
$$

$$
c_{k+1} = \xi_{k+1} + e^{-\omega \Delta t}\,(c_k - \xi_{k+1})
$$

Explicit Euler would grow the CP by $1 + \omega\Delta t$ per cycle instead of $e^{\omega\Delta t}$.
Over a 1 s step at 500 Hz that comes out about 2 % short of $e^{\omega T}$. The CP then missed the
foothold by about 3 mm per step, and the error added up.

### 1.2 Reference outputs (TSID CoM task)

$$
c_{ref} = (c_x,\ c_y,\ z_c), \qquad
\dot c_{ref} = \omega\,(\xi - c), \qquad
\ddot c_{ref} = \omega^2\,(c - p)
$$

The vertical velocity and acceleration are 0. `WalkPlanner::toWorld()` transforms these into the
world frame: positions with the full support-frame transform, velocities and accelerations with
its rotation only.

---

## 2. Receding-horizon ZMP (Stages 1 + 2)

Each phase has a CP target $\xi_d$ (section 4) and an end time. In every cycle, `planCycle()`
solves for the **constant ZMP** that brings the CP to the target in the remaining time $t_{rem}$.
With $p$ constant, $\xi(t_{rem}) = p + b\,(\xi_{fb} - p)$ where $b = e^{\omega t_{rem}}$.
Setting $\xi(t_{rem}) = \xi_d$ and solving for $p$ (`zmpForTargetCP`):

$$
\boxed{\,p = \frac{\xi_d - b\,\xi_{fb}}{1 - b}, \qquad b = e^{\omega\,t_{rem}}\,}
$$

- With no disturbance, this gives the same ZMP every cycle: the once-per-phase solution.
- If the CP drifts (the ZMP was clamped, a step ended late, or the robot lags the plan), the
  correction is spread over the rest of the phase, with no jump.
- **Clamp:** $p$ is limited to the support area of the current phase:
  - in single support, the stance sole in the ankle frame, $x \in$ [`sole_x_min`, `sole_x_max`]
    and $|y| \le$ `sole_y_half`;
  - with both feet down, the box around both soles.

  The CP may then miss $\xi_d$. The foothold adaptation (section 5) takes over that error.
- **Hold:** as $t_{rem} \to 0$ the solution diverges (it divides by $1 - b \to 0$). Once
  $t_{rem} \le$ `zmp_hold_time`, the last ZMP is held, still clamped to the limits of this cycle.
  The limits change at liftoff, from the double-support box to the stance sole. A late step
  ($t_{rem} = 0$) also holds the ZMP.

```cpp
this->cp_fb_ = this->cp_ + this->cp_feedback_gain_ * (this->cp_meas_ - this->cp_);
if (this->t_rem_ > this->zmp_hold_time_)
{
    this->zmp_d_ = this->zmpForTargetCP(this->cp_fb_, this->t_rem_);
}
this->zmp_d_ = this->zmp_d_.cwiseMax(zmp_min).cwiseMin(zmp_max);
this->ComputeCP(this->zmp_d_);
this->ComputeCoM(this->cp_);
```

---

## 3. CP feedback

The plan is a model. The robot's real CP is measured every cycle (`WalkPlanner::measureCP`), from
the estimated state $(q, v)$ and Pinocchio's CoM and CoM velocity, in the support frame:

$$
\xi_{meas} = c_{meas} + \frac{\dot c_{meas}}{\omega}
$$

It is used in two places, both weighted by $k$ = `cp_feedback_gain` (0 = open loop, 1 = fully
measured):

1. **ZMP (every cycle).** The ZMP of section 2 is solved from a blended CP, not the planned one:

   $$
   \xi_{fb} = \xi + k\,(\xi_{meas} - \xi)
   $$

   The planned state $\xi$, $c$ itself is still integrated from the model. Only the ZMP decision
   sees the measurement.

2. **Touchdown re-seed (each support switch).** The planned CoM and CP are pulled toward the
   measured ones, so the plan cannot drift away from the robot over many steps
   (`blendTowardMeasured`):

   $$
   c \leftarrow c + k\,(c_{meas} - c), \qquad \xi \leftarrow \xi + k\,(\xi_{meas} - \xi)
   $$

---

## 4. Phases and CP targets

`WalkPlanner::planWalking()` runs the phases `INIT` → `STEP` → `STOP` → `REST`, where `STEP` is
double support followed by single support. Each phase gives the LIP a CP target, a remaining time
and ZMP limits (`phaseTarget()`).

| Phase | CP target $\xi_d$ (support frame) | $t_{rem}$ | ZMP limits | Ends |
|---|---|---|---|---|
| `INIT` | $b_c$ (stance ankle + CoM bias) | $T - t$ | both soles | at $T$ → first step (right foot swings), or `STOP` |
| `STEP`, double support | $\xi_{start}$ | $t_{ds} - t$ | both soles | at $t_{ds}$ → trailing foot lifts off |
| `STEP`, single support | $E_{ss}\,\xi_{start}$ | $\max(T - t, 0)$ | stance sole | touchdown (event based) or $T$ (time based) |
| `STEP`, closing step | $b_c + \tfrac12 f_{nom}$ (between the feet) | $\max(T - t, 0)$ | stance sole | touchdown / $T$ → `STOP` |
| `STOP`, `REST` | $b_c + \tfrac12 f_{other}$ (middle of the feet) | $T - t$ | both soles | `STOP` at $T$ → `REST`; `REST` stays |

**CoM bias.** $b_c = c_0 - \tfrac12 f_{right,0}$ is how far the CoM sits from the middle of the
feet at `reset()`. The plan keeps that offset instead of pulling the CoM onto the geometric middle.

### 4.1 Periodic gait CP (Stage 3b)

The CP at the start of single support, $\xi_{start}$, is chosen so that the gait is **periodic**:
every step starts from the same CP, mirrored left / right. It is computed in the heading axes,
where $s$ is the step length and $y_f$ the lateral foothold of the coming step (± stance width),
and then rotated by the heading error $\psi_e$ (section 7):

$$
\xi_{start} = R(\psi_e)
\begin{pmatrix}
\dfrac{s\,(1 + E_{ds})}{2\,(E_{ds}E_{ss} - 1)} \\[2ex]
\dfrac{y_f\,(1 + E_{ds})}{2\,(1 + E_{ds}E_{ss})}
\end{pmatrix}
$$

**Derivation.**

1. In single support the ZMP is at the stance ankle (the origin), so the touchdown CP is
   $E_{ss}\,\xi_{start}$.
2. In the new frame, on the landed foot $f = (s, y_f)$, this CP is $E_{ss}\xi_{start} - f$.
3. Double support then holds the ZMP midway between the feet, at $-f/2$, for $t_{ds}$:

   $$
   \xi' = -\tfrac{f}{2} + E_{ds}\left(E_{ss}\,\xi_{start} - f + \tfrac{f}{2}\right)
   $$

4. Periodicity requires $\xi'_x = \xi_{start,x}$, and $\xi'_y = -\xi_{start,y}$ because the
   lateral side alternates. Solving for $\xi_{start}$ gives the formula above.

With $t_{ds} = 0$ ($E_{ds} = 1$) it reduces to the classic touchdown CP
$E_{ss}\xi_{start} = \left(\frac{s\,E_{ss}}{E_{ss}-1},\ \frac{y_f\,E_{ss}}{E_{ss}+1}\right)$.

**Lateral margin.** $E_{ss}$ is large (about $10^2$ for $T$ = 1 s), so $\xi_{start,y}$ is tiny.
A lateral CP error at the start of single support grows by $E_{ss}$ by touchdown, while the ZMP
can only move $\pm$`sole_y_half` (2 cm). This is why the foothold has to take over the correction.

---

## 5. Foothold adaptation (Stage 3)

The ZMP can only move on the sole. The foothold takes over the rest of the correction
(`adaptFoothold`, run every single-support cycle).

1. **Predicted CP at the end of the step,** with the ZMP of this cycle kept for the remaining time
   (`getPredictedEndCP`):

   $$
   \xi_{end} = p + e^{\omega\,t_{rem}}\,(\xi_{fb} - p)
   $$

2. **Foothold for that CP.** The next step starts like the planned one if the foot stands where
   the CP target is, relative to the nominal foothold $f_{nom}$:

   $$
   f = \xi_{end} - (\xi_d - f_{nom})
   $$

   If the ZMP is not clamped, then $\xi_{end} = \xi_d$ and $f = f_{nom}$.

3. **Limits.** $e^{\omega t_{rem}}$ is large early in the step, so the prediction is noisy. In the
   heading axes the foothold is limited to:
   - $|x| \le$ `max_step_length`;
   - $y \in y_f \pm$ `foothold_max_width_change`.

   It is also rate limited to `foothold_max_rate`·$\Delta t$ per cycle (a smooth swing target), and
   frozen after phase `foothold_freeze_phase` and while late (the foot must land where it is
   heading).

Disabled with `foothold_adaptation: false`.

---

## 6. Swing foot trajectory

`computeSwingTrajectory` gives the swing foot at phase
$\phi = t / T \in [0, 1]$, in the support frame. It goes from the liftoff position $x_0$ to the
foothold $f$ on a **cycloid**, with zero velocity and acceleration at both ends, and adds a
vertical **bump** of height $h$ = `step_height` that peaks at $\phi = 0.5$:

$$
\gamma(\phi) = \frac{2\pi\phi - \sin 2\pi\phi}{2\pi}, \qquad
\beta(\phi) = \frac{h}{2}\,(1 - \cos 2\pi\phi)
$$

$$
x(\phi) = x_0 + \gamma(\phi)\,\Delta, \qquad
\Delta = f - x_0 - (0,\,0,\,d_{land}), \qquad
x_z \mathrel{+}= \beta(\phi)
$$

The velocity and acceleration use $\dot\phi = 1/T$:
$\dot x = \frac{\gamma'}{T}\Delta$ and $\ddot x = \frac{\gamma''}{T^2}\Delta$, where
$\gamma' = 1 - \cos 2\pi\phi$ and $\gamma'' = 2\pi \sin 2\pi\phi$. The bump terms are added the
same way.

- **Landing depth** $d_{land}$ = `swing_landing_depth`: the trajectory ends this far below the
  foothold, so the foot touches the ground before $\phi = 1$ while still moving down. The impact is
  easier for the contact estimator to detect. The current yaml value is 0; with 0.01 the early
  touchdowns ended the steps about 15 % early.
- **Yaw:** it follows the same cycloid, from the liftoff yaw to the heading of the step:
  $\psi(\phi) = \psi_0 + \gamma(\phi)\,\mathrm{wrap}(\psi_e - \psi_0)$.

### 6.1 Late touchdown descent

This applies only in event mode. If the step reaches $T$ without a touchdown, the step waits and
the foot keeps descending below the foothold (`computeLateDescent`), as in tsid_examples. The
speed ramps up from 0 to $v$ = `late_descent_speed` over $T_r$ = `late_descent_ramp_time`. With
$t$ the time since $T$:

$$
d(t) =
\begin{cases}
\dfrac{v\,t^2}{2T_r} & t < T_r \\[1.5ex]
\dfrac{v\,T_r}{2} + v\,(t - T_r) & t \ge T_r
\end{cases}
\qquad d \le d_{max} = \texttt{late\_max\_depth}
$$

While late, the ZMP is held (because $t_{rem} = 0$) and the foothold is frozen.

---

## 7. Heading (turning)

The controller turns the turn command into a yaw rate $\dot\psi_{cmd}$ (times `max_yaw_rate`). At
each support switch (`switchSupport`), the heading advances by one step period, limited both per
step and relative to the new stance foot's yaw $\psi_s$:

$$
\Delta\psi = \mathrm{clamp}\!\left(\dot\psi_{cmd}\,(T + t_{ds}),\ \pm\Delta\psi_{max}\right), \qquad
\psi_e = \mathrm{clamp}\!\left(\mathrm{wrap}(\psi + \Delta\psi - \psi_s),\ \pm\Delta\psi_{max}\right)
$$

$$
\psi \leftarrow \psi_s + \psi_e, \qquad \Delta\psi_{max} = \texttt{max\_turn\_per\_step}
$$

- The **nominal step** is defined in the heading axes, $s_h = (s,\ \pm w)$ with $w$ the stance
  width measured at `reset()`, and rotated into the support frame:
  $f_{nom} = R(\psi_e)\,s_h$. The periodic CP $\xi_{start}$ is rotated the same way (section 4.1).
- The **swing foot yaw** goes to $\psi_e$ on the cycloid (section 6).
- The **torso yaw** is the mean of the two feet's yaw references:
  $\psi_{base} = \psi_s + \tfrac12\psi_{swing}$, with its rate and acceleration as feedforward.
- While a turn is commanded, the controller sets the step length to 0, so the robot turns in place.

---

## 8. Support switch and step end

At touchdown (event based), or at $T$ (time based), `endStep()` runs:

1. **`switchSupport`**:
   - The landed foot becomes the stance foot, and a new support frame ${}^oM_s$ is placed on its
     ankle (yaw only).
   - The LIP state ($c$, $\xi$, $p$, $\xi_d$, $\xi_{fb}$) is re-expressed in the new frame,
     $x \leftarrow R\,x + t$ with ${}^{s}M_{s,old} = {}^oM_s^{-1}\,{}^oM_{s,old}$. The motion
     itself does not change.
   - The trailing foot's liftoff position and yaw are taken from the current state.
   - The heading of the coming step is updated (section 7), and the nominal foothold
     $f_{nom} = R(\psi_e)\,s_h$ is set.
   - Touchdown re-seed: the CP is measured in the new frame and blended in (section 3).
2. **Stop handling:**
   - If a stop was requested, the next step is the **closing step**: $s = 0$, so the swing foot
     lands beside the stance foot and the feet end side by side.
   - After the closing step lands, the phase goes to `STOP`.
3. Otherwise **double support** starts ($t_{ds} > 0$), then the next swing.

---

## 9. Per-cycle algorithm

```text
planWalking(q, v):                                   # every control cycle, support frame
    ξ_meas ← c_meas + ċ_meas / ω                     # measureCP
    (ξ_d, t_rem, p_min, p_max) ← phaseTarget()       # section 4
    ξ_fb ← ξ + k (ξ_meas − ξ)                        # section 3
    if t_rem > zmp_hold_time:
        p ← (ξ_d − e^{ω t_rem} ξ_fb) / (1 − e^{ω t_rem})   # section 2
    p ← clamp(p, p_min, p_max)
    ξ ← p + e^{ωΔt} (ξ − p);   c ← ξ + e^{−ωΔt} (c − ξ)    # section 1.1
    t ← t + Δt

    if double support and t ≥ t_ds:      end double support (trailing foot lifts off)
    elif single support:
        if touchdown or (t ≥ T and time based):  endStep()      # section 8
        else:                                     adaptFoothold() # section 5
    elif t ≥ T:                          endOfPhase()  # INIT → STEP / STOP, STOP → REST

    late ← single support and t ≥ T                  # event based only
    if single support: swing ← cycloid(t / T) (+ late descent)   # section 6
    toWorld()                                        # CoM, swing foot, torso yaw → world
```

---

## 10. Parameters (`config/tsid_wbc.yaml`, section `LIPPlanner`)

| Key | Value | Meaning |
|---|---|---|
| `Tswing` | 1.0 s | swing time $T$ |
| `sampling_frequency` | 500 Hz | planner rate, $\Delta t$ = 2 ms; must equal the controller rate |
| `com_height` | 0.45 m | default $z_c$; replaced by the measured CoM height at `reset()` |
| `double_support_time` | 0.2 s | $t_{ds}$ at the start of each step (0 = instant switch) |
| `max_step_length` | 0.3 m | step length clamp (step = `walk_vx` · $T$) and foothold x limit |
| `step_height` | 0.05 m | swing bump height $h$ |
| `swing_landing_depth` | 0.0 m | $d_{land}$, end of the swing below the foothold |
| `sole_x_min` / `sole_x_max` / `sole_y_half` | −0.04 / 0.12 / 0.02 m (header defaults, not in the yaml) | single-support ZMP limits (ankle frame) |
| `cp_feedback_gain` | 0.3 | $k$, measured-CP blend (ZMP and touchdown re-seed) |
| `zmp_hold_time` | 0.03 s | ZMP held for the last part of a phase |
| `foothold_adaptation` | true | section 5 on / off |
| `foothold_max_width_change` | 0.05 m | lateral range around the nominal foothold |
| `foothold_max_rate` | 0.5 m/s | foothold rate limit |
| `foothold_freeze_phase` | 0.8 | no foothold change after this phase |
| `late_descent_speed` | 0.2 m/s | late descent speed $v$ |
| `late_descent_ramp_time` | 0.05 s | ramp $T_r$ |
| `late_max_depth` | 0.05 m | deepest late target $d_{max}$ |
| `max_turn_per_step` | 0.2 rad | $\Delta\psi_{max}$ |
| `hip_width` | 0.27 m | not used: the stance width is measured at `reset()` |

The controller-side settings are in `config/controllers.yaml`, section `wbc_controller`:
- `walk_vx` gives the step length;
- `max_yaw_rate` gives the turn rate;
- `use_time_base` chooses between step ends at $T$ and at the detected touchdown.

The event-based contact FSM that calls `touchdown()` is in `WBCController::computeWalkCommand`
([wbc_controller.cpp](../src/control/wbc_controller.cpp)).
