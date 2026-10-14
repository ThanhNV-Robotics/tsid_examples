# Event-based walking control

Reference documentation of the event-based walking controller of
`biped_walking`: each step ends on a touchdown **event** detected by the
sensorless contact estimator, not on a timer, and the CoM / capture-point (CP)
plan is kept consistent with whatever the actual step timing is.

Related documents:

- [contact_fusion.md](contact_fusion.md): the contact estimator (momentum
  observer, probability models, Kalman fusion) that produces the contact
  events used here.
- [tsid.md](tsid.md): the TSID whole-body controller, tasks and contacts.

Code locations are given as `file:function`; line numbers drift, function
names do not.

---

## 1. Overview

### 1.1 What the controller does

| Function | Where | Stage |
|---|---|---|
| Steps end on touchdown events from the contact fusion (early, on time or late) | `WalkPlanner::planWalking` | 0 (contact events), 4 |
| The swing foot keeps descending when the ground is lower than expected (late touchdown) | `WalkPlanner::planWalking`, `FootPlacement::StepSwingPlanning` | 0 |
| ZMP recomputed every tick from the current CP and the remaining phase time | `WalkPlanner::planWalking` | 1 |
| The robot's measured CP is fed back into the plan | `WalkPlanner::measureCP`, `planWalking`, `enterStance` | 2 |
| The next foothold follows the predicted CP when the ZMP is saturated | `WalkPlanner::planWalking` | 3 |
| Each step starts with a double-support phase that absorbs timing errors | `WalkPlanner::planWalking`, `enterStance`, `clampToDoubleSupport` | 3b |

"Stage 0" is the event logic that existed before the CP replanning (touchdown
confirmation, late state); Stages 1 to 3b were added on top of it.

### 1.2 One control tick (1 kHz), `demos/biped_walking.cpp`

```
sim.getRobotSensorValues()
  └─► Cheat_StateEstimator::estimate()        base pose/twist (MuJoCo), joints; ground-truth contact (evaluation)
        └─► robotStateToPinocchio()           q, v
ContactFusion::computeDisturbanceTorque(q, v, τ)        foot forces f̂_z, sole heights p_z
ContactFusion::fuseContactProbability(s_φ, φ)          ŝ per foot   (s_φ, φ from the planner's PREVIOUS tick)
WalkPlanner::planWalking(..., contact_flags = ŝ)       events, CP / ZMP / CoM plan, next foothold
  └─► com_task reference ← getCoMrefW / getCoMvelRefW / getCoMaccRefW
FootPlacement::StepSwingPlanning(...)                  swing trajectory (cycloid + late descent)
contact switch on events:
  stanceChanged() (touchdown) ─► tsidTaskParser::endSwing(landing foot)     rigid contact added
  liftoffEvent()             ─► tsidTaskParser::startSwing(trailing foot)  contact removed (ramped), swing task added
tsid.computeProblemData() ─► HQP solve ─► τ ─► sim.setControl(τ)
```

The fusion uses the planner's state of the previous tick (its gait schedule
is the phase prior), and the planner uses the fusion's output of the current
tick: the loop is causal.

### 1.3 Gait cycle

```
standing ──(T_stand)──► initial CoM shift (both feet, one t_swing)
    │
    ▼
 ┌─────────────────────────── one step ───────────────────────────┐
 │ touchdown event                                                  │
 │   ──► DOUBLE SUPPORT (double_support_time, both feet loaded)     │
 │   ──► liftoff event (trailing foot)                              │
 │   ──► SINGLE SUPPORT (t_swing, swing foot in the air)            │
 │         ├─ early touchdown event (φ ≥ 0.7, contact held 25 ms) ──┼──► next step
 │         ├─ φ = 1 and swing foot in contact ──────────────────────┼──► next step
 │         └─ φ = 1, no contact ─► LATE: foot keeps descending      │
 │                 └─ contact held late_confirm_time ───────────────┼──► next step
 └──────────────────────────────────────────────────────────────────┘
```

Default timing: single support 0.8 s, double support 0.2 s (1 s per step),
`vx` = 0.15 m/s, step length `vx · t_swing` = 0.12 m (≤ `max_step_length`).

## 2. Contact events in the walk planner

All in `WalkPlanner::planWalking`, single-support branch.

### 2.1 Inputs

`contact_flags = [left, right]`, chosen in `biped_walking.cpp`:

```cpp
const bool use_estimated_contact = true;   // ŝ from ContactFusion; false: simulator ground truth (> 20 N)
```

Only the **swing** foot's flag is read (`contact_flags[left_stance ? 1 : 0]`);
the stance leg is the planner's own decision.

### 2.2 Liftoff seen, contact timer

```
if (!swing_contact) swing_lifted_ = true;                       // latched until the next step
swing_contact_time_ = (swing_lifted_ && swing_contact) ? +dt : 0  // uninterrupted contact since the foot came down
```

Right after liftoff the swing foot is still in contact (the TSID contact force
ramps down over `contact_transition_time` = 0.15 s, and `ŝ` drops 50 to 150 ms
later); `swing_lifted_` prevents that contact from being read as a touchdown.

### 2.3 Step-end rules

| Case | Condition | Result |
|---|---|---|
| Early touchdown | `swing_lifted_` and contact held `touchdown_confirm_time` (25 ms) and φ ≥ `touchdown_phase_min` (0.7) | step ends now |
| On time | φ ≥ 1 and swing foot in contact | step ends now |
| Late | φ ≥ 1 and no contact | late state (2.4) |

The 25 ms confirmation exists because the swing foot touches the floor
lightly (2 to 10 N) at φ ≈ 0.95 to 0.98; ending the step on that light touch
moved the full weight onto an unloaded foot before the CoM was ready and made
the robot fall.

### 2.4 Late state

- Entered at φ ≥ 1 without contact; `isLate()`, `getLateTime()`.
- The support leg is kept. `FootPlacement::StepSwingPlanning` holds the swing
  target's x, y at the landing point and lowers z: the speed ramps from 0 to
  `late_descent_speed` (0.2 m/s) over `late_descent_ramp_time` (0.05 s), then
  stays constant, down to `late_max_depth` (0.02 m) below the planned landing
  height. Position, velocity and acceleration of that profile go to the swing
  task.
- The step ends when the contact holds `late_confirm_time` (5 ms).
- `late_touchdown_timeout`: 0 (default) waits for the touchdown; > 0 ends the
  step after that time anyway (the TSID contact is then added in the air) and
  prints a warning.

Why the ramp: a velocity jump at the start of the late state made the swing
task's damping demand a large downward acceleration at once; a foot already
resting on the floor was driven into it with 70 to 120 N every step (at
1 m/s), which built up a lateral oscillation.

On flat ground the swing foot ends every step resting on the floor but
unloaded (0 N, fused probability 0.39 to 0.40 < `p_on` 0.5), so almost every
step goes late for about 30 ms and ends on the contact event once the descent
has loaded the foot. With the double-support phase (Stage 3b) this is harmless.

## 3. CoM and capture-point planning

### 3.0 Capture-point control: background and code

This subsection explains the method the planner is built on and shows where
each idea lives in `src/planning/walk_planner.cpp`. The rest of section 3
describes the stages in detail.

#### The model: linear inverted pendulum (LIPM)

With the CoM at a constant height `z_c` and negligible angular momentum about
the CoM, the horizontal CoM motion depends only on the ZMP `p` (the centre of
pressure of the ground reaction force):

```
c̈ = ω² (c − p),        ω = √(g / z_c)
```

For this robot `z_c` ≈ 0.545 m, so ω ≈ 4.24 s⁻¹ (time constant 1/ω ≈ 0.24 s).
The CoM is pushed **away** from the ZMP like a falling pendulum: the further it
is from the ZMP, the faster it accelerates away.

In the planner, `w` is ω, set from the CoM height when walking starts
(`setInitCom`):

```cpp
this->w = std::sqrt(this->g / this->zc_); // keep w consistent with the new height
```

#### Splitting the dynamics: the capture point

Define the capture point (CP), also called the divergent component of motion
(DCM):

```
ξ = c + ċ / ω
```

The second-order pendulum then splits into two first-order systems:

```
ċ = ω (ξ − c)          stable:   the CoM converges to the CP
ξ̇ = ω (ξ − p)          unstable: the CP runs away from the ZMP
```

- The CoM follows the CP with time constant 1/ω whatever happens, so only the
  CP has to be controlled.
- The ZMP pushes the CP: the CP moves directly away from the ZMP, at a speed
  proportional to their distance. With the ZMP behind the CP the CP
  accelerates forward; with the ZMP exactly on the CP the CP stops.

These two equations are integrated every tick (explicit Euler, `dt_` = 1 ms):

```cpp
double WalkPlanner::CoM_dynamics(double cxi, double xc)
{
    // Input: cxi: capture point input, xc: CoM position
    double d_xc;
    d_xc = this->w* (cxi - xc);
    return d_xc;
}

double WalkPlanner::CP_dynamics (double p, double cxi)
{
    double d_cxi;
    d_cxi = this->w*(cxi - p);
    return  d_cxi;
}

void WalkPlanner::computeCoM(double cxi_x, double cxi_y)
{
    d_xc_ = CoM_dynamics(cxi_x, this->xc_);
    d_yc_ = CoM_dynamics(cxi_y, this->yc_);
    xc_ += d_xc_*dt_;
    yc_ += d_yc_*dt_;
}

void WalkPlanner::computeCP (double zmp_x, double zmp_y)
{
    double d_cxi_x{0}, d_cxi_y{0};
    d_cxi_x = CP_dynamics(zmp_x, this->cxi_x_);
    d_cxi_y = CP_dynamics(zmp_y, this->cxi_y_);

    this->cxi_x_ += d_cxi_x*dt_;
    this->cxi_y_ += d_cxi_y*dt_;
}
```

At the end of `planWalking`, the commanded ZMP drives the CP, and the CP
drives the CoM:

```cpp
        // compute Capture Point
        this->computeCP(zmp_x_d_, zmp_y_d_);

        // calculate CoM
        this->computeCoM(cxi_x_, cxi_y_);
```

The robot's measured CP uses the same definition, from the CoM position and
velocity of the last TSID problem (Stage 2):

```cpp
void WalkPlanner::measureCP(const tsid::robots::RobotWrapper &robot_wrapper, const pinocchio::Data &data)
{
    // CoM position and velocity of the robot (computeAllTerms of the last TSID
    // problem), in the planning frame; CP of the LIPM: c + dc / w
    com_meas_ = worldToPoint(robot_wrapper.com(data).head<2>());
    const Eigen::Vector2d vcom = worldToDir(robot_wrapper.com_vel(data).head<2>());
    cxi_meas_ = com_meas_ + vcom / this->w;
}
```

#### Why it is called the capture point

If the ZMP could be placed exactly at ξ and held there (by stepping there or
shifting the weight there), the CP would stay put (ξ̇ = 0) and the CoM would
converge to it and come to rest: the robot is **captured**.

The ZMP can only lie inside the support polygon: the stance sole in single
support, the area between both feet in double support. This gives a simple
stability criterion:

- **CP inside the support polygon:** the robot can stop without stepping, by
  moving its ZMP onto the CP.
- **CP outside it:** no ZMP placement can stop it; the robot must step, so
  that the CP falls inside the new support.

In the planner the support polygon is the clamp applied to every ZMP
command:

```cpp
Eigen::Vector2d WalkPlanner::clampToSole(const Eigen::Vector2d &p) const
{
    return Eigen::Vector2d(std::clamp(p.x(), sole_x_min_, sole_x_max_),
                           std::clamp(p.y(), -sole_y_half_, sole_y_half_));
}

Eigen::Vector2d WalkPlanner::clampToDoubleSupport(const Eigen::Vector2d &p) const
{
    // Band from the stance ankle (planning-frame origin) to the trailing ankle:
    // along the band, both ankles plus the sole half width; across it, the
    // sole's toe/heel extent (the band runs mostly sideways, across = forward)
    const Eigen::Vector2d ab = trail_foot_;
    const double len = ab.norm();
    if (len < 1e-6) return clampToSole(p);
    const Eigen::Vector2d u = ab / len;
    Eigen::Vector2d n(-u.y(), u.x());
    if (n.x() < 0.0) n = -n; // across axis pointing forward
    const double along = std::clamp(p.dot(u), -sole_y_half_, len + sole_y_half_);
    const double across = std::clamp(p.dot(n), sole_x_min_, sole_x_max_);
    return along * u + across * n;
}
```

In single support the lateral range is only ±20 mm (`sole_y_half_`); in
double support it spans both feet, about 30 cm.

#### Steering the CP with the ZMP

With a constant ZMP `p` over a time `T`, the CP equation has a closed-form
solution:

```
ξ(T) = p + e^(ωT) (ξ(0) − p)
```

The CP moves along the line through `p`, and its distance from `p` grows by
`e^(ωT)`. Two classic control laws use this.

> **The planner computes the ZMP with one law only: (a), the end-of-step law
> in receding-horizon form.** Law (b) is shown for comparison, because it is
> the other common way to do capture-point control and appears in the
> literature; it is not implemented. The note after (b) shows how the two are
> related.

**(a) End-of-step control** (Englsberger 2011): choose the constant ZMP that
brings the CP to a desired value `ξ_d` at the end of the phase:

```
p = (ξ_d − e^(ωT) ξ) / (1 − e^(ωT))
```

The planner uses this law. Stage 1 recomputes it every tick with `T` = the
**remaining** time of the current phase (receding horizon), so a disturbance
is corrected immediately instead of at the next step. Stage 2 uses a blend of
the planned and the measured CP for `ξ`:

```cpp
        measureCP(robot_wrapper, data);
        const double cxi_fb_x = cxi_x_ + cp_feedback_gain_ * (cxi_meas_.x() - cxi_x_);
        const double cxi_fb_y = cxi_y_ + cp_feedback_gain_ * (cxi_meas_.y() - cxi_y_);
        // Remaining time of the current phase (DS: to liftoff, SS: to touchdown)
        const double t_rem = in_ds_ ? std::max(t_ds_ - ds_time_, 0.0)
                                    : this->t_swing * (1.0 - std::min(phi_CoM, 1.0));
        if (t_rem > zmp_hold_time_) {
            const double b_rem = std::exp(this->w * t_rem);
            const Eigen::Vector2d p((cxi_xd_ - b_rem * cxi_fb_x) / (1.0 - b_rem),
                                    (cxi_yd_ - b_rem * cxi_fb_y) / (1.0 - b_rem));
            const Eigen::Vector2d p_c = in_ds_ ? clampToDoubleSupport(p) : clampToSole(p);
            zmp_x_d_ = p_c.x();
            zmp_y_d_ = p_c.y();
        }
```

**(b) Proportional CP tracking** (**not used in this planner**, for comparison
only): follow a CP reference trajectory `ξ_ref(t)` with

```
p = ξ_ref − ξ̇_ref / ω + k (ξ − ξ_ref),     k > 1
```

Substituting into `ξ̇ = ω (ξ − p)` gives the error dynamics `ė = ω (1 − k) e`,
stable for `k > 1`: the ZMP is pushed past the CP error so the CP is pulled
back. This is the form of the DCM controller in Englsberger et al. (2015).

**How (a) relates to (b), and why the planner uses (a).** Rearranging (a)
with `B = e^(ω T_rem)`:

```
p = (ξ_d − B ξ) / (1 − B)
  = ξ + (ξ − ξ_d) / (B − 1)
```

So (a) is also proportional feedback on the CP error: it places the ZMP
beyond the CP, on the side away from the target, by an amount proportional
to the distance from the CP to the target. The differences:

| | (a) End-of-step, receding horizon (**used**) | (b) Proportional tracking (not used) |
|---|---|---|
| Error measured against | one **end point** `ξ_d` (CP at liftoff or touchdown) | a full **reference trajectory** `ξ_ref(t)` |
| Feedback gain | `1 / (e^(ω T_rem) − 1)`, **rising** toward the end of the phase: about 0.03 at the start of single support, → ∞ as `T_rem` → 0 | constant `k > 1` |
| Needs | `ξ_d` and the remaining time | `ξ_ref(t)` and `ξ̇_ref(t)` at every instant |
| Behaviour | weak correction early (the error has time to be fixed), stronger near the end | uniform correction along the trajectory |
| Weak point | ill-conditioned as `T_rem` → 0, hence the ZMP hold for the last `zmp_hold_time` (30 ms) | needs a reference trajectory consistent with the step timing |

Reasons for (a) in this planner:

1. **It fits event-based timing.** It only needs the target at the end of the
   current phase and the time left. When a touchdown comes early or late, the
   next phase simply starts with its own target and remaining time; nothing
   else has to be regenerated. With (b), every timing change would require
   recomputing the whole reference trajectory `ξ_ref(t)`.
2. **It was already in the planner.** The original planner computed this
   formula once per step; Stage 1 only changed it to "every tick, with the
   remaining time", which turned it into a feedback law.
3. **It is closed-loop with Stage 2.** Using the blended measured CP for `ξ`
   gives the same kind of feedback controller as (b).

(b) would become the better choice if the planner generated an explicit,
smooth CP trajectory (for example with ZMP shapes that vary within a phase,
as in Englsberger et al. 2015): its constant gain avoids the end-of-phase
gain blow-up, at the cost of generating and regenerating `ξ_ref`.

Both laws share the same limit: **`p` must stay inside the support polygon.**
When the law asks for a ZMP outside it, the ZMP is clamped and the CP error
can no longer be fully corrected through the ZMP. That is what happened in the
instant-switch gait, with only ±20 mm of lateral ZMP range under one sole.

#### Stepping when the ZMP is not enough (Stage 3)

When the ZMP is clamped, the CP at touchdown will miss its target; the next
foot is then placed where the gait resumes from the predicted CP. The
prediction is the closed-form solution above with the commanded (clamped) ZMP:

```cpp
            const Eigen::Vector2d zmp(zmp_x_d_, zmp_y_d_);
            const Eigen::Vector2d cxi_fb(cxi_fb_x, cxi_fb_y);
            const Eigen::Vector2d cxi_end = zmp + std::exp(this->w * t_rem) * (cxi_fb - zmp);
            const Eigen::Vector2d cxi_td_rel_next = cxi_ss_end - foot_nominal;
            // Limits in the walking-heading axes: step length, lateral range
            // around the nominal width (keeps the feet apart, no crossing)
            Eigen::Vector2d q = R_err.inverse() * (cxi_end - cxi_td_rel_next);
```

followed by the clamps and the rate limit of section 3.5.

#### Walking with the CP

A walking gait is a sequence of ZMP locations (mostly on the stance foot) and
CP targets that chain from step to step. During single support the ZMP sits
near the stance ankle and the CP runs away from it toward where the next foot
will land; at touchdown the next foot becomes the support and the CP is
pushed onward. For a steady, periodic gait, the CP at the start and end of
each step can be solved so that every step looks the same; with a
double-support phase the ZMP moves across from the old foot to the new one
while both feet are loaded, a gentle hand-over instead of a jump. These are
the targets `ξ₀` and `ξ_ss_end` (derivation in section 3.2):

```cpp
        const double b = std::exp(this->w * this->t_swing);       // E_ss
        const double e_ds = std::exp(this->w * this->t_ds_);      // E_ds
        const Eigen::Vector2d cxi_start = R_err * Eigen::Vector2d(
            step_length * (1.0 + e_ds) / (2.0 * (e_ds * b - 1.0)),
            side * wd_hip * (1.0 + e_ds) / (2.0 * (1.0 + e_ds * b)));
        const Eigen::Vector2d cxi_ss_end = b * cxi_start;
        // Active target: start of SS while in DS, touchdown CP while in SS
        const Eigen::Vector2d cxi_d = in_ds_ ? cxi_start : cxi_ss_end;
```

#### From the CP plan to TSID

The planner's ZMP is a *planned* ZMP; nothing commands it directly. TSID
receives the CoM reference (position, velocity and, as feedforward, the LIPM
acceleration `ω² (c − p)`), and its contact forces realise that acceleration;
the real ZMP follows the planned one as long as TSID tracks the CoM well. That
is why Stage 2 compares the planned CP with the measured one.

```cpp
Vector3d WalkPlanner::getCoMaccRef() const
{
    return Vector3d(w * w * (xc_ - zmp_x_d_), w * w * (yc_ - zmp_y_d_), 0.0);
}
```

In `biped_walking.cpp` (world frame):

```cpp
        com_sample.setValue(walk_planner.getCoMrefW());
        com_sample.setDerivative(walk_planner.getCoMvelRefW());
        com_sample.setSecondDerivative(walk_planner.getCoMaccRefW());
        com_task->setReference(com_sample);
```

#### Summary: CP concepts and their code

| CP concept | Code |
|---|---|
| `ω = √(g / z_c)` | `setInitCom` (`w`) |
| `ξ̇ = ω (ξ − p)`, `ċ = ω (ξ − c)` integrated | `CP_dynamics`, `CoM_dynamics`, `computeCP`, `computeCoM` |
| measured CP `ξ = c + ċ/ω` | `measureCP` |
| support polygon (ZMP limits) | `clampToSole` (SS), `clampToDoubleSupport` (DS) |
| end-of-step control, receding horizon (the only ZMP law used) | ZMP block of `planWalking` (Stage 1) |
| feedback on the measured CP | blend `cxi_fb_*` (Stage 2) |
| stepping when the ZMP is not enough | foothold block of `planWalking` (Stage 3) |
| periodic CP targets | `cxi_start` (ξ₀), `cxi_ss_end` (Stage 3b) |
| CoM reference for TSID | `getCoMrefW`, `getCoMvelRefW`, `getCoMaccRefW` |

#### Why the instant-switch gait failed, in CP terms

With 1 s of single support, `e^(ωT)` ≈ 69. To make the CP cross the 25 cm
stance width in one step with the ZMP fixed at the stance ankle, the CP had to
start almost on the stance ankle and accelerate exponentially, reaching the
new foot exactly at the step end at about 1 m/s. Any delay of the foot switch
let it overshoot the new foot by about 1 mm per millisecond. Once the CP was
beyond the new sole's outer edge (outside its support polygon), only a step
could capture it, and the next swing foot was on the wrong side.

The double-support phase fixes this twice over: the CP target at touchdown is
76 mm short of the new foot instead of 4 mm (margin for a late touchdown), and
during double support the support polygon spans both feet, so the ZMP has room
to steer the CP to where the next single support needs it.

### 3.1 Model and frames

Linear inverted pendulum, constant CoM height `z_c`:

```
ω = √(g / z_c)            (z_c = 0.545 m → ω = 4.24 s⁻¹)
ξ = c + ċ / ω             capture point (divergent component of motion)
ξ̇ = ω (ξ − p)            p = ZMP
ċ = ω (ξ − c)
```

`computeCP` and `computeCoM` integrate these (explicit Euler, 1 ms). The
planning frame is anchored to the stance ankle at each touchdown
(`enterStance` → `reanchor`); `R_err` rotates the nominal step from the
reference heading into the stance foot's axes. Outputs go to the world frame
through `pointToWorld` / `dirToWorld` (`getCoMrefW`, `getZMPrefW`,
`getCapturedPointrefW`, `getNextFootW`).

### 3.2 Nominal targets: periodic gait with double support (Stage 3b)

ZMP constant at the stance ankle during single support (SS) and at the
midpoint between the feet during double support (DS). For step length `s`,
stance width `w_h` (next foot at `(s, side·w_h)`), `E_ss = e^(ω t_swing)`,
`E_ds = e^(ω double_support_time)`:

```
ξ₀       = ( s (1 + E_ds) / (2 (E_ds E_ss − 1)),   side · w_h (1 + E_ds) / (2 (1 + E_ds E_ss)) )
ξ_ss_end = E_ss · ξ₀
```

`ξ₀`: CP at the start of SS (relative to the stance foot), target of the DS.
`ξ_ss_end`: CP at the end of SS (touchdown), target of the SS. For
`double_support_time = 0` they reduce to the single-support solution
`(s/(b−1), side·w_h/(b+1))` and `b·ξ₀`, `b = e^(ω t_swing)`.

Why this matters (lateral, `w_h` = 0.256 m):

| SS / DS | CP at touchdown, short of the new ankle | CP speed at touchdown | late-touchdown tolerance before DS |
|---|---|---|---|
| 1.0 s / 0 (old gait) | 4 mm | 1.07 m/s | about none |
| 0.8 s / 0.2 s (default) | 76 mm | 0.76 m/s | about 70 ms |
| 0.7 s / 0.3 s | 95 mm | 0.69 m/s | about 110 ms |

In the old gait every millisecond of delay at the step end pushed the CP about
1 mm past the new foot; a touchdown 19 ms late put it on the outer edge of the
new sole, where neither the ZMP nor the next foothold (on the other side) could
recover it.

### 3.3 Stage 1: receding-horizon ZMP

Every tick, the constant ZMP that takes the CP from its current value to the
active target in the **remaining** phase time:

```
T_rem = DS: double_support_time − ds_time      SS: t_swing · (1 − φ)
p     = (ξ_target − e^(ω T_rem) ξ) / (1 − e^(ω T_rem))
p     = clampToDoubleSupport(p)  (DS)   or   clampToSole(p)  (SS)
```

Held (not recomputed) when `T_rem ≤ zmp_hold_time` (0.03 s): the formula is
ill-conditioned as `T_rem → 0`. In the late state `T_rem = 0`, so the last ZMP
is held. Undisturbed, the result equals the once-per-phase solution; a CP
error is spread over the rest of the phase instead of being frozen at its start.

Limits: `clampToSole` = stance sole box (`sole_x_min` −0.04, `sole_x_max`
0.12, `sole_y_half` 0.02 m, in the stance-ankle frame).

### 3.4 Stage 2: measured-CP feedback

`measureCP` reads the CoM position and velocity from the last TSID problem
(`robot.com(data)`, `robot.com_vel(data)`), converts them to the planning frame
(`worldToPoint`, `worldToDir`) and forms `ξ_meas = c + ċ/ω`. The CP used in the
Stage 1 formula is a blend:

```
ξ_fb = ξ_plan + k (ξ_meas − ξ_plan)        k = cp_feedback_gain (0.3)
```

At every touchdown (`enterStance`) the planned CoM and CP are pulled toward
the measured ones by the same blend, so plan and robot cannot drift apart over
many steps; the CoM reference given to TSID stays continuous for `k < 1`.
The planned CP and CoM are still integrated from the commanded ZMP.

Effect on the stable walk: lateral CoM error at touchdown 13.8 → 8.8 mm.
Higher gains (0.6, 1.0) did not help the failing cases and made them worse
(the ZMP has only ±20 mm of lateral authority).

### 3.5 Stage 3: CP-based foothold adaptation

During SS, before `foothold_freeze_phase` (0.8):

```
ξ_end  = p + e^(ω T_rem) (ξ_fb − p)          predicted CP at touchdown with the commanded ZMP
p_next = ξ_end − (ξ_ss_end − foot_nominal)    touchdown CP relative to the next foot = nominal
```

- While the ZMP is not clamped, `ξ_end = ξ_ss_end` and `p_next = foot_nominal`:
  the foothold stays nominal (verified: identical step widths with and without
  adaptation on flat ground).
- When the ZMP is clamped, the foothold takes over the part of the correction
  the ZMP cannot do: `(e^(ω T_rem) − 1)(p_unclamped − p_clamped)`.
- `e^(ω T_rem)` reaches about 30 early in a step, so the result is clamped in
  the heading axes (`|x| ≤ max_step_length`, lateral within
  `±foothold_max_width_change` = 5 cm of the nominal width, no crossing),
  rate-limited (`foothold_max_rate` 0.5 m/s), frozen after φ = 0.8 and reset to
  nominal at each step start.
- `FootPlacement` takes the landing point from `getNextFootW()`, so the swing
  trajectory follows.

### 3.6 Stage 3b: double-support phase

- `enterStance` (touchdown) re-anchors the frame to the new stance foot,
  records the trailing ankle (`trail_foot_`, planning frame), sets
  `in_ds_ = true`, `ds_time_ = 0`. With `double_support_time = 0` the liftoff
  event fires on the same tick (old instant-switch behaviour).
- During DS the SS phase `phi_CoM` waits; `ds_time_` runs. Target `ξ₀`.
- ZMP limits in DS (`clampToDoubleSupport`): the band from the stance ankle to
  the trailing ankle; along the band both ankles plus `sole_y_half`, across it
  the sole's toe/heel extent (the band runs mostly sideways). About 30 cm of
  lateral authority instead of ±2 cm.
- At `ds_time_ ≥ double_support_time`: `in_ds_ = false`, `liftoffEvent()` true
  for one tick, SS starts (`getSwingPhase()` ramps from 0).
- Getters: `inDoubleSupport()`, `liftoffEvent()`; `stanceChanged()` is the
  touchdown event; `getSwingPhase()` is 0 during DS.

### 3.7 What is not replanned

- The initial CoM shift (standing → left foot, one `t_swing` long) still uses
  the once-per-phase ZMP formula.
- Phase durations (`t_swing`, `double_support_time`) are fixed; only their end
  is event-triggered (touchdown) or stretched (late state). Step-timing
  adaptation (Khadiv et al.) is not implemented.
- The CoM height is constant (`z_c` at the start of walking).

## 4. Integration in `biped_walking.cpp`

- **Contact switch** (`enable_swing` block): `stanceChanged()` →
  `task_parser.endSwing(tsid, swing_frame, ...)` (landing foot: swing task
  removed, rigid contact added at its current pose, max normal force
  restored); `liftoffEvent()` → `task_parser.startSwing(tsid, frame)`
  (trailing foot: contact removed with the 0.15 s force ramp, swing task
  added). Between the two, both feet are in contact (DS).
- **Contact fusion schedule**: during DS (or before stepping) both feet are
  scheduled in contact with subphase 0.5; during SS the stance foot is in
  contact and the swing foot is not, subphase = `getSwingPhase()`.
- **Step duration** from the YAML (`t_swing`), printed at start.
- **Console events**: `[Swing] t=... touchdown <foot>`, `... liftoff <foot>`,
  `... late touchdown: <foot> not in contact at the end of the step`,
  `[WalkPlanner] late touchdown timeout ...` (only if the timeout is > 0),
  `[ContactFusion] contact state accuracy ...` at exit.
- **CSV** (`record/biped_standing_joint_control.csv`), columns used by the
  evaluation: `swing_leg`, `swing_phase`, `late_touchdown`, `double_support`,
  `plan_com`, `plan_zmp`, `plan_cp`, `meas_cp`, `com_meas`, `swing_land`,
  `lfoot_meas`, `rfoot_meas`, `base_rpy`, `sole_z`, `ground_z`, `f_est_z`,
  `f_gt_z`, `p_*`, `s_hat`, `s_gt`.
- **Environment variables** (opt-in, no effect when unset):
  `TSID_HEADLESS=1` creates all windows hidden
  (`src/sim/Robot_Simulator.cpp`); `TSID_SIM_TIME=<s>` stops the demo at that
  simulated time.

## 5. Parameters

All under `walk_planner: foot_placement:` in
[tsid_config.yaml](../config/tsid_config.yaml); read by `WalkPlanner` and
`FootPlacement`. Missing keys keep the defaults in the headers.

| Key | Default (YAML) | Stage | Meaning |
|---|---|---|---|
| `t_swing` | 0.8 s | 3b | single-support duration (also the initial CoM shift duration) |
| `double_support_time` | 0.2 s | 3b | double support at the start of each step; 0 = instant switch |
| `vx`, `ramp_time` | 0.15 m/s, 1 s | — | forward speed command and its ramp |
| `max_step_length` | 0.15 m | 3 | step length limit (`vx · t_swing` and foothold clamp) |
| `stepHeight` | 0.05 m | — | swing apex height |
| `z_offset` | 0 | 0 | swing landing target relative to the ground (< 0 presses the foot down; −5 mm made the robot fall) |
| `touchdown_phase_min` | 0.7 | 0 | earliest phase for an early touchdown |
| `touchdown_confirm_time` | 0.025 s | 0 | contact confirmation for an early touchdown |
| `late_confirm_time` | 0.005 s | 0 | contact confirmation in the late state |
| `late_touchdown_timeout` | 0 | 0, 4 | 0 = wait for the touchdown; > 0 = end the late step after that time |
| `late_descent_speed` | 0.2 m/s | 0 | late descent speed |
| `late_descent_ramp_time` | 0.05 s | 0 | ramp of the late descent speed |
| `late_max_depth` | 0.02 m | 0 | deepest late target below the planned landing (0 = no limit); must exceed the largest expected step-down, see 8 |
| `zmp_hold_time` | 0.03 s | 1 | ZMP held below this remaining phase time |
| `cp_feedback_gain` | 0.3 | 2 | measured-CP blend (0 = open loop) |
| `foothold_adaptation` | true | 3 | enable the foothold adaptation |
| `foothold_max_width_change` | 0.05 m | 3 | lateral range around the nominal width |
| `foothold_max_rate` | 0.5 m/s | 3 | foothold rate limit |
| `foothold_freeze_phase` | 0.8 | 3 | foothold frozen after this phase |

Other relevant values: `swing_foot_task.contact_transition_time` 0.15 s (TSID
force ramp at liftoff), sole limits `sole_x_min/max`, `sole_y_half` (header
defaults −0.04, 0.12, 0.02 m), the contact fusion parameters
([contact_fusion.md](contact_fusion.md)), and in `biped_walking.cpp`
`use_estimated_contact`, `enable_swing`, `T_stand` (3 s).

## 6. Results so far

All runs headless, flat ground unless stated, contact estimate unless stated.

### 6.1 Development history (why each stage exists)

| Configuration | Result | Cause |
|---|---|---|
| Old gait (1 s SS, instant switch), late timeout 5 ms | stable 463 s | every step ended by the 5 ms timeout: a timer, not events |
| Old gait, late timeout 0 (wait for contact) | falls at step 3 to 5 | late steps of 12 to 30 ms push the CP past the new foot (section 3.2) |
| Old gait, `z_offset` −5 / −10 mm | falls at step 1 / 2 | pressing the swing foot down disturbs single support |
| + Stage 1 | flat stable (CoM error 13.8 mm); failing cases unchanged | |
| + Stage 2 (k 0.3) | flat stable (CoM error 8.8 mm); failing cases unchanged | ZMP authority ±20 mm |
| + Stage 3 | flat unchanged (footholds nominal); failing cases unchanged | the CP leaves the new sole on the outside, the next swing foot is on the other side |
| Shorter steps 0.5 / 0.4 s | fall in 2 to 3 steps, also with ground truth | gait tuning (swing gain, contact ramp) matched to 1 s steps |
| + Stage 3b (0.8 / 0.2 s) | see 6.2 | |

### 6.2 Current controller (Stage 3b, defaults)

| Test | Steps | Step endings | Max roll / pitch | Other |
|---|---|---|---|---|
| Flat, 120 s | 112 | 112 contact events (late about 31 ms), 0 timeouts | 1.54° / 0.86° | CoM error at touchdown 8.3 mm; ZMP never at the sole limit in SS |
| Flat, ground-truth contact, late timeout 0, 60 s | 54 | 54 contact events (late about 28 ms) | 1.46° / 0.79° | |
| Regression: `double_support_time` 0, `t_swing` 1.0, timeout 5 ms, 60 s | 55 | 55 timeouts | 0.70° / 0.24° | identical to Stage 3 (code path unchanged when DS = 0) |
| Box 1.5 cm (x 0.63 to 1.63 m), 35 s | 30 | 2 early events (toe on the box edge), 28 late contact events, longest 92 ms (step-down) | 2.61° / 2.09° | feet land at 14.5 mm on the box and back at 0 after it; ground estimate follows |
| Box 3 cm, `late_max_depth` 0.05, 30 s | 5 | — | falls at the step-up (t = 9.2 s) | foot landed straddling the edge (heel on the floor, toe on the box, pitched 20°), rolled off during DS |
| Flat, late timeout 0, `z_offset` −5 mm | 3 | early events at φ 0.87 to 0.89 | falls | not needed any more |

Contact estimation accuracy with DS: 93.6 to 94.1 % (about 96.5 % without DS:
two more transitions per step where the fused state lags the ground truth).

## 7. Tests

### 7.1 Tools

```
# headless run + summary (copies the log and CSV to record/eval_<name>.{log,csv})
~/miniconda3/envs/torch_env/bin/python scripts/eval_walking.py --run --time 60 --name <name>

# summary of an existing run
~/miniconda3/envs/torch_env/bin/python scripts/eval_walking.py --csv record/eval_<name>.csv --log record/eval_<name>.log

# contact estimation figure and metrics for a run
~/miniconda3/envs/torch_env/bin/python scripts/plot_data.py -f record/eval_<name>.csv
```

`eval_walking.py` reports: survival time and QP failure, number of steps,
step endings (early event / late → contact event / late → timeout), late
durations, max roll and pitch while walking, lateral CoM error one tick before
each touchdown (planned vs measured), share of single support with the planned
ZMP at the lateral sole limit, step width at touchdown vs the nominal width,
and the contact estimation accuracy. Each run takes about 10 to 20 s of wall
time per 10 s of simulated time; the config is read at start, so YAML changes
need no rebuild.

### 7.2 Tests that have been run (repeat after any change)

| ID | Setup | Pass criterion | Last result |
|---|---|---|---|
| T1 | defaults, flat, `--time 120` | no QP failure; all steps end on contact events, 0 timeouts; max roll < 2°; ZMP limit 0 % | pass (6.2) |
| T2 | `double_support_time: 0`, `t_swing: 1.0`, `late_touchdown_timeout: 0.005`, `--time 60` | identical to the pre-Stage-3b baseline (roll 0.70°, CoM error 8.8 mm, 55 timeouts) | pass |
| T3 | as T1 with `use_estimated_contact = false` (rebuild) | as T1 | pass (ground truth, 54 events) |
| T4 | `step_box` enabled (1.5 cm), `--time 35` | no fall; feet at ≈ 14.5 mm on the box and ≈ 0 after; no timeouts | pass |
| T5 | contrast: `double_support_time: 0`, `t_swing: 1.0`, timeout 0 | falls within the first steps (shows what Stage 3b fixes) | falls at step 3 |

For T4 the sequence of landings (surface, timing) can be listed from the CSV:
foot `x`, `sole_z_*` and `late_touchdown` at each change of `swing_leg`.

### 7.3 Proposed tests (not run yet)

| ID | Test | How | What it shows |
|---|---|---|---|
| P1 | Lateral push recovery | drag the torso in the viewer, or add a scripted external force (e.g. `xfrc_applied` on the torso for 0.1 s, 20 to 60 N sideways) | whether Stage 3 widens the steps and Stage 2/3b recover; report the largest impulse recovered |
| P2 | Double-support duration sweep | `double_support_time` 0.1, 0.15, 0.2, 0.3 with `t_swing` = 1 − DS | smallest DS that stays event-based and stable; tolerance margin |
| P3 | Feedback gain sweep on the current gait | `cp_feedback_gain` 0, 0.3, 0.6, 1.0 | whether stronger feedback helps now that DS gives ZMP authority (it did not before) |
| P4 | Speed range | `vx` 0 (stepping in place), 0.05, 0.1, 0.15, 0.2 | step lengths up to `max_step_length`; sagittal limits |
| P5 | Step-down depth | box heights 1, 1.5, 2 cm with `late_max_depth` above the drop | late-touchdown duration vs drop; the CP margin of section 3.2 |
| P6 | Unexpected step-up height | box heights 1, 2, 2.5, 3 cm | largest step-up before the straddling failure of 6.2 |
| P7 | Foothold adaptation on/off | `foothold_adaptation: false` vs true under P1 | contribution of Stage 3 |
| P8 | Sensor noise | add noise to joint torques / velocities in the demo | robustness of the contact events (force estimate) and of the CP feedback |
| P9 | Formula checks (offline) | small Python script: simulate the LIPM with the Stage 3b targets for one period | `ξ₀`, `ξ_ss_end` periodic, and equal to the single-support formulas for DS = 0 |
| P10 | Long run | T1 with `--time 600` | drift of heading, foothold or plan over hundreds of steps |

## 8. Limitations

- **Partial footholds.** The controller is terrain-blind; a foot landing across
  an edge (3 cm box) is accepted as a contact and TSID treats it as a flat
  rigid contact. Needs swing clearance near touchdown, a foothold check (e.g.
  foot pitch at touchdown) or terrain perception.
- **`late_max_depth` with no timeout.** The late descent stops at
  `late_max_depth` below the planned landing; with `late_touchdown_timeout: 0`
  a drop deeper than that waits forever (the CP drifts). Keep `late_max_depth`
  above the largest expected step-down (for example 0.05 m) or set a timeout.
- **Every step is slightly late on flat ground** (about 30 ms): the swing foot
  ends its trajectory unloaded. Harmless with DS; a deliberate landing phase in
  the swing trajectory would make the touchdown on time.
- **Fixed phase durations.** No step-timing optimisation; the late state only
  extends single support.
- **Simulation-only state.** The base pose and twist come from MuJoCo
  (`Cheat_StateEstimator`), which the CP feedback and the foot heights rely on.
- **Contact estimation accuracy** drops to about 94 % with DS; the fusion
  parameters were tuned without it.
- **Shorter steps** (< 0.8 s) need the gait retuned (swing task gain, contact
  transition time) before event timing can be tested on them.

## 9. File map

| File | Content |
|---|---|
| `src/planning/walk_planner.cpp`, `include/planning/walk_planner.h` | events, late state, CP / ZMP / CoM plan, Stages 1 to 3b |
| `src/planning/foot_placement.cpp`, `include/planning/foot_placement.h` | swing trajectory, late descent |
| `demos/biped_walking.cpp` | tick order, contact switch on events, fusion schedule, logging, `TSID_SIM_TIME` |
| `src/control/parse_tsid_tasks.cpp` | `startSwing` / `endSwing` (TSID contacts and swing task) |
| `src/estimation/contact_fusion.cpp` | contact events (see [contact_fusion.md](contact_fusion.md)) |
| `src/sim/Robot_Simulator.cpp` | `TSID_HEADLESS` |
| `config/tsid_config.yaml` | all parameters of section 5 |
| `scripts/eval_walking.py` | headless run and walk summary |
| `scripts/plot_data.py` | contact estimation figure and metrics |

## 10. References

- G. Bledt, P. M. Wensing, S. Ingersoll, S. Kim, "Contact Model Fusion for
  Event-Based Locomotion in Unstructured Terrains," ICRA 2018 (contact
  events, early / late touchdown).
- J. Englsberger, C. Ott, M. A. Roa, A. Albu-Schäffer, G. Hirzinger, "Bipedal
  walking control based on Capture Point dynamics," IROS 2011 (end-of-step CP
  control, Stage 1).
- J. Englsberger, C. Ott, A. Albu-Schäffer, "Three-dimensional bipedal walking
  control based on Divergent Component of Motion," IEEE T-RO 2015 (DCM
  feedback, double support, Stages 2 and 3b).
- J. Pratt, J. Carff, S. Drakunov, A. Goswami, "Capture Point: A step toward
  humanoid push recovery," Humanoids 2006.
- T. Koolen, T. de Boer, J. Rebula, A. Goswami, J. Pratt, "Capturability-based
  analysis and control of legged locomotion," IJRR 2012.
- M. Khadiv, A. Herzog, S. A. A. Moosavian, L. Righetti, "Step timing
  adjustment: A step toward generating robust gaits," Humanoids 2016 (not
  implemented; possible extension).

Citations are from memory; check the exact form of the control laws in the
papers before citing them.
