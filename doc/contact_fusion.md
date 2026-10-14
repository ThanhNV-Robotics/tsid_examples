# Contact fusion: sensorless foot contact estimation

This document describes the foot contact estimator as implemented in this
repository: what it computes, how it is wired into the biped walking
controller, how to evaluate it, and what is known about its behaviour. The
design discussion and the original plan are in
[contact_fusion_plan.md](contact_fusion_plan.md); this file describes the code
as it is.

Reference: G. Bledt, P. M. Wensing, S. Ingersoll, S. Kim, *"Contact Model Fusion
for Event-Based Locomotion in Unstructured Terrains"*, ICRA 2018. Section and
equation numbers below refer to that paper.

---

## 1. Goal

The robot has no foot contact sensor. Contact must come from proprioception
only:

- joint encoders (q, q̇) and the base state,
- applied joint torques (motor current on the robot, `actuator_force × gear`
  in MuJoCo),
- the gait schedule of the walk planner.

The estimator produces, for each foot, a contact probability `P(c)` and a
binary contact state `ŝ`. The walk planner uses `ŝ` to detect touchdown, and
through the planner's step event TSID switches its rigid contacts. The
simulator's true contact state is used **only** for evaluation.

## 2. Pipeline (one 1 kHz control tick)

```
RobotSensor ─► Cheat_StateEstimator::estimate() ─► q, v   (base from MuJoCo, joints from sensors)
                         │
                         └─► ground-truth contact force per foot (evaluation only)

ContactFusion::computeDisturbanceTorque(q, v, τ)          §III
   ├─ discrete-time momentum observer       ─► τ̂_d (nv)
   ├─ per-foot wrench (damped solve)        ─► f̂_z per foot
   └─ forward kinematics                    ─► sole height p_z per foot

ContactFusion::fuseContactProbability(s_φ, φ)             §IV
   ├─ u = P(c | s_φ, φ)   phase prior (planner state of the previous tick)
   ├─ z = [P(c | p_z); P(c | f_z)]
   ├─ Kalman predict / update                ─► P(c) per foot
   └─ hysteresis                             ─► ŝ per foot

WalkPlanner::planWalking(..., contact_flags = ŝ)          §VI
   └─ touchdown confirmed for t_delay ─► step event ─► TSID contact switch
```

All of it lives in `ContactFusion`
([contact_fusion.h](../include/estimation/contact_fusion.h),
[contact_fusion.cpp](../src/estimation/contact_fusion.cpp)), with the generic
filter in `KalmanFilter`
([kalman_filter.h](../include/estimation/kalman_filter.h)).

## 3. Contact force from the momentum observer (§III)

### 3.1 Disturbance torque

With the floating-base dynamics `M v̇ + C v + g = Sᵀτ + τ_d` and the
generalized momentum `p = M v`, the discrete-time observer of eq. (10) is

```
γ = exp(-2π f_c Δt)            f_c = 15 Hz (constructor argument), Δt = 1 ms
β = (1 - γ) / (γ Δt)
x_k   = β p_k + Sᵀτ_k + C_kᵀ v_k - g_k
y_k   = γ y_{k-1} + (1 - γ) x_k          first-order low-pass filter
τ̂_d,k = β p_k - y_k
```

No joint acceleration is needed. Implementation notes:

- `M` (CRBA, symmetrized), `C` (`computeCoriolisMatrix`, satisfies Ṁ = C + Cᵀ)
  and `g` are computed by Pinocchio in a `pinocchio::Data` owned by
  `ContactFusion`. `tsid.data()` cannot be used: at this point of the tick it
  still holds the previous state, and TSID overwrites it later.
- `τ` is the **applied** torque, `RobotSensor::actuator_state.torquej`, not the
  commanded one (they differ when an actuator saturates).
- The filter starts at the first tick with `y_0 = x_0`, which gives the
  quasi-static value `τ̂_d = g - Sᵀτ - Cᵀv` without a start-up transient.
- The MJCF joints have zero damping, armature and friction loss, so no
  compensation of those terms is needed in simulation. On the robot, gearbox
  friction and rotor inertia would appear as a fake disturbance.

### 3.2 Per-foot wrench

Assuming every disturbance on a leg's joints comes from its foot (§III-C):

```
ŵ_i = (S_i J_iᵀ)⁻¹ S_i τ̂_d          ŵ_i = [f; m], world-aligned axes at the foot frame origin
```

- `J_i`: 6 × nv frame Jacobian of the foot frame in `LOCAL_WORLD_ALIGNED`, so
  `f̂_z = ŵ_i(2)` is the vertical force. The force part does not depend on the
  reference point, so the ankle frame gives the same `f̂_z` as the sole.
- `S_i`: the velocity indices of the joints between the base and the foot,
  found by walking `model.supports` (the free-flyer is excluded). For a 6-DoF
  leg `S_i J_iᵀ` is 6 × 6.
- **Damped least squares instead of an exact solve.** At the end of stance the
  trailing leg is nearly straight and `S_i J_iᵀ` is close to singular. An exact
  solve (inverse or QR) divides the observer noise by the tiny singular value;
  estimates of −196 558 N and +2 062 N were observed right after a stance
  switch, producing false contacts in mid-swing. The solve now uses the SVD
  with damping λ = `wrench_damping_` = 0.02:
  `ŵ = V diag(σ / (σ² + λ²)) Uᵀ S_i τ̂_d`. Well-conditioned directions are
  almost unchanged (σ = 0.1 → factor 0.96; the force directions have σ ≈ 0.3 to
  0.7), near-singular ones are suppressed.
- `f̂_z` is clamped to `[0, 3 m g]` (about 0 to 610 N here): the ground only
  pushes, and walking stays far below 3 m g.

### 3.3 Accuracy

On a flat-ground walk (before the damped solve), the sum of both estimates
averaged **202.7 N** against **m g = 203.2 N** (robot mass 20.72 kg from the
MJCF). In single support the estimate follows the simulator's contact force;
after liftoff it decays to zero over about 0.1 s (the 15 Hz filter).

The MuJoCo `<touch>` sensors are **not** a usable reference: they only measure
contact points inside their site volume and read 30 to 50 % low with large
chatter (their sum averaged 147.5 N for the same 203 N load).

## 4. Contact probability models (§IV)

Three models map one quantity each to a probability of contact. The member
names follow the code.

### 4.1 Phase prior `P(c | s_φ, φ)` (eq. in §IV-A)

```
P = ½ ( s_φ [erf((φ - μ_c0)/(σ√2)) + erf((μ_c1 - φ)/(σ√2))]
      + (1 - s_φ) [2 + erf((μ_c0 - φ)/(σ√2)) + erf((φ - μ_c1)/(σ√2))] )
```

`calculate_phase_contact_probability(phi, scheduled_contact)`, with
μ_c0 = 0, μ_c1 = 1 and σ = `sigma_phase_c_` = 0.1 (variance 0.01; the paper's
Table I uses σ² = 0.05, σ ≈ 0.22).

Gait schedule from the walk planner (`biped_walking.cpp`):

| Planner state | Left foot | Right foot | Subphase φ |
|---|---|---|---|
| `DSt` (standing, initial CoM shift) | s_φ = 1 | s_φ = 1 | 0.5 |
| `LSt` (left stance) | s_φ = 1 | s_φ = 0 | `getSwingPhase()` |
| `RSt` (right stance) | s_φ = 0 | s_φ = 1 | `getSwingPhase()` |

The planner state is the one of the **previous** tick: the planner itself
consumes `ŝ` in the current tick, so this keeps the loop causal.

### 4.2 Foot height `P(c | p_z)` (eq. 20)

```
P(c | p_z) = ½ [1 + erf((μ_zg - (p_z - z_g)) / (σ_zg √2))]
```

- `p_z` (`getFootHeight(i)`): world height of the **lowest sole corner**. The
  corners are the TSID contact points of the foot
  (`tsidTaskParser::ContactInfo::contact_points`, passed with
  `setSoleCorners`). The lowest corner rather than the ankle or the sole
  centre makes heel and toe strikes on a tilted foot count as soon as one
  corner reaches the ground.
- `z_g` (`getGroundHeight(i)`): **adaptive ground height** under the foot
  (§IV-B.1), from the previous tick's contact state. A foot in contact stands
  on the ground, so its own sole height is its ground; a swinging foot expects
  the ground at the mean sole height of the feet in contact (terrain assumed
  locally flat around the stance foot). With no foot in contact the last value
  is kept. On the 1.5 cm step this makes the height model agree with the
  force once a foot stands on the step, instead of reading "in the air".
- `μ_zg` = `muy_zg_` = 0.01 m is a **margin above the estimated ground**, and
  `σ_zg` = `sigma_zg_` = 0.005 m. With no margin a flat foot resting on the
  ground would always give 0.5; with 1 cm it gives P ≈ 0.98 on the ground, 0.5
  at 1 cm above it and ≈ 0 at the 5 cm swing apex.
- Consequence of the adaptive ground: while a foot is in contact its height
  model reads ≈ 0.98, so liftoff is detected by the phase prior and the force
  alone (section 7.4). A variant that freezes the ground at each touchdown was
  tested offline and was worse (97 % accuracy, liftoff 23 to 31 ms early, false
  switches).
- `p_z` uses the base height of `q`. In simulation that is exact; on the robot
  it inherits the drift of the base-height estimate.

### 4.3 Foot force `P(c | f_z)` (eq. 22)

```
P(c | f_z) = ½ [1 + erf((f̂_z - μ_fc) / (σ_fc √2))]
```

`μ_fc` = `muy_fc_` = 20 N, `σ_fc` = `sigma_fc_` = 10 N: about 0.02 with the foot
in the air, 1 under a stance load of 100 to 200 N.

## 5. Kalman fusion (§IV, eq. 12 to 26)

State `x = [P_L(c), P_R(c)]`, built by `makeContactFilter(n_feet)`:

| Matrix | Value | Paper |
|---|---|---|
| A | 0 | eq. 16: no state carried over |
| B | I | eq. 16: input `u` = phase prior |
| C (= H) | [I; I] | eq. 25: height and force measurements |
| Q (= Σ_w) | 0.998 I | Sec. V-A tuned value |
| R (= Σ_v) | blkdiag(0.841 I, 0.5 I) | height: Sec. V-A value; force: lowered from 0.930 (see below) |

Each tick `fuseContactProbability` calls `predict(u)` then `update(z)` with
`z = [P(c|p_z) L, R; P(c|f_z) L, R]` (all heights first, then all forces, to
match `C`) and clamps the result to [0, 1].

Because `A = 0` and all matrices are diagonal, the filter is exactly a
per-foot inverse-variance weighted average:

```
P(c) = (u / 0.998 + z_h / 0.841 + z_f / 0.5) / (1/0.998 + 1/0.841 + 1/0.5)
     ≈ 0.24 u + 0.28 z_h + 0.48 z_f
```

The paper's covariances give almost equal weights (0.31 / 0.36 / 0.33). The
force variance `kRForce_` was lowered from 0.930 to 0.5 because the observer
force is accurate in simulation (section 3.3): in an offline replay of a walk
onto the 1.5 cm step, with the adaptive ground, it raised the accuracy from
98.0 / 98.4 % to 98.4 / 98.6 % and brought liftoff within ±5 ms. On the real
robot the force estimate will be noisier and this weight should be retuned.

**Contact decision** (eq. 26 with hysteresis): `ŝ` switches on when
`P(c) > p_on_` = 0.5 and off when `P(c) < p_off_` = 0.4.

## 6. Use in the walk planner (§VI)

> The walk planner side (gait cycle with double support, early / late
> touchdown, CP replanning) is documented in
> [event_based_control.md](event_based_control.md); this section covers how
> the contact estimate enters it.

`WalkPlanner::planWalking(..., contact_flags)` takes the contact state as an
argument. `biped_walking.cpp` selects it with

```cpp
const bool use_estimated_contact = true;   // false: simulator ground truth
```

The planner uses the flags to start stepping (both feet in contact) and to
detect the swing foot's touchdown. A touchdown ends the step early only when

- the swing foot has left the ground during this step,
- it has been back in contact **without interruption for
  `touchdown_confirm_time`** (paper Sec. VI `t_delay`), and
- the step phase is at least `touchdown_phase_min`;

otherwise the step ends at φ = 1 if the swing foot is in contact. If it is
not (ground lower than expected), the planner enters the **late** state (paper
Fig. 10):

- the support leg is kept (`isLate()`, `getLateTime()`),
- `FootPlacement` holds the swing target's x, y at the landing point and lowers
  z, ramping the speed up to `late_descent_speed` over `late_descent_ramp_time`,
  down to `late_max_depth` below the planned landing height. The ramp matters:
  a velocity jump at the start of the late state makes the swing task's damping
  demand a large downward acceleration at once, and a foot already resting on
  the floor (the usual flat-ground case: at φ = 1 it is on the floor but
  unloaded) is driven into it. At 1 m/s without ramp this produced 70 to 120 N
  kicks every step and a growing lateral oscillation,
- the step ends when the contact holds for `late_confirm_time`: the foot keeps
  stretching down until the touchdown. `late_touchdown_timeout` > 0 adds an
  optional safety net (after that time the step ends anyway, with the TSID
  contact added in the air, and a warning is printed); 0 disables it. The
  capture point diverges while the ZMP stays on the old stance foot, so a very
  long wait is itself a risk.

Without this, a missed touchdown added a rigid TSID contact at the airborne
foot's pose and the robot dropped onto it.

All keys are in [tsid_config.yaml](../config/tsid_config.yaml) under
`walk_planner: foot_placement:`:

```yaml
touchdown_phase_min: 0.7
touchdown_confirm_time: 0.025 # [s]
late_confirm_time: 0.005      # [s]
late_touchdown_timeout: 0.0   # [s], 0 = wait for the touchdown
late_descent_speed: 0.2       # [m/s]
late_descent_ramp_time: 0.05  # [s]
late_max_depth: 0.02          # [m], 0 = no limit
```

`biped_walking` prints `[Swing] ... late touchdown` when a step goes late and
logs the state in the CSV column `late_touchdown`.

**Why the confirmation is needed.** The swing trajectory brings the foot onto
the floor at φ ≈ 0.975, where it touches lightly (2 to 10 N) until the step
ends. The simulator's 20 N contact threshold never fires in that window, so
with ground-truth flags every step ends exactly at φ = 1 and the walk is
stable. The fusion correctly detects the light touch 5 to 10 ms after it
starts; without confirmation the planner ended the step there (φ = 0.963,
then 0.87 on the next step), TSID moved the full weight onto a foot carrying
10 N while the CoM and capture point were still short of their planned
end-of-step state, and the robot fell within two steps. With 25 ms of
confirmation the light touch reaches φ = 1 before it is accepted, while a real
early contact (for example on a step) is still accepted 25 ms after it starts.

## 7. Ground truth and evaluation

### 7.1 Ground truth

`Cheat_StateEstimator` sums `mj_contactForce` over every contact between a
foot body (`left_ankle_pitch_link`, `right_ankle_pitch_link`) and anything
that is not part of the robot (floor, terrain), in the world frame
(`getContactForces()`). Its `contact_flags` are "vertical force > 20 N". These
are simulator queries, not robot sensors; they are used for plots, logs and
accuracy only (and by the planner if `use_estimated_contact = false`).

### 7.2 Real-time plots (`biped_walking`)

| Window | Lines |
|---|---|
| Foot contact force | `L_est`, `R_est` (f̂_z) vs `L_gt`, `R_gt` |
| Foot height (contact fusion) | `z_L`, `z_R` (lowest sole corner) and the adaptive ground `ground_L`, `ground_R` |
| Phase contact probability | `P_L`, `P_R` and the schedule `s_L`, `s_R` |
| Force contact probability | `P_L`, `P_R` vs `gt_L`, `gt_R` |
| Height contact probability | `P_L`, `P_R` vs `gt_L`, `gt_R` |
| Fused contact probability (Kalman) | `P_L`, `P_R` vs `gt_L`, `gt_R` |
| Contact state: estimated vs ground truth | `ŝ` and ground truth per foot (right foot shifted by 1.5) |

The plots sample once per rendered frame (about 60 Hz); use the CSV for
millisecond timing. At exit the demo prints the share of ticks where `ŝ`
matches the ground truth for each foot, counted while walking.

### 7.3 CSV log and offline analysis

`record/biped_standing_joint_control.csv` contains, per foot (`_L`, `_R`):
`f_est_z`, `f_gt_z`, `sole_z`, `ground_z`, `p_phase`, `p_height`, `p_force`,
`p_fused`, `s_hat`, `s_gt`; plus `swing_leg`, `swing_phase` and
`late_touchdown`. These are all the inputs of the fusion, so it can be replayed
offline with other parameters.

[scripts/plot_data.py](../scripts/plot_data.py) plots the force comparison and
a contact-estimation figure (probabilities, `ŝ` vs the reference, sole height
and estimated ground per foot), and prints from `--t-start` (default 3 s, the
start of walking): accuracy, mean and max touchdown and liftoff delay (each
reference edge matched to the nearest estimated edge within ±0.3 s) and the
number of switches.

The **reference** is, by default, the physical contact: ground-truth force
above 1 N (`--ref-level`), with state changes shorter than 20 ms
(`--debounce`) removed. The logged `s_gt` flags (`--reference flag`) are a
poor reference for timing: their 20 N threshold is crossed in the middle of
the liftoff force ramp, they flicker there (61 switches against 17 physical
ones in one run), and a light touch below 20 N never registers.

```
~/miniconda3/envs/torch_env/bin/python scripts/plot_data.py
```

### 7.4 Observed timing

Against the physical reference (section 7.3):

| Edge | Delay of `ŝ` | Notes |
|---|---|---|
| Touchdown, flat ground | 0 to 8 ms | the force rises within one observer time constant |
| Touchdown, toe clip on a step edge | not detected (by design) | the force goes 50 → 18 → 3 to 6 → 0 N: a graze, not a load-bearing contact; detecting it would end a step on a foot hanging on an edge |
| Liftoff | −5 to +12 ms (replayed with the current settings) | phase prior and force; the height model reads ≈ 1 while in contact (adaptive ground) |

Before the adaptive ground and the force weight, liftoff measured 50 to 150 ms
late against the 20 N flags and about 30 ms early against the physical
contact. Most of the apparent "delay" against the ground-truth plot came from
the 20 N reference itself, not from the estimator. The liftoff timing does
not affect the planner: it only needs the swing foot to have left the ground
before φ = `touchdown_phase_min`.

## 8. Limitations and next steps

- **Unexpected step-ups.** The adaptive ground follows the terrain once a foot
  stands on it, but the first contact with a higher surface is, by
  construction, above the expected ground: only the force can detect it.
  Light toe clips on a step edge (`step_box` in
  [scene_floatingbase_12dof_v2.xml](../models/biped/mjcf/scene_floatingbase_12dof_v2.xml))
  are therefore not detected, which is intended; they also show that the
  swing foot's clearance near the end of the swing is low.
- **Late contact with ground-truth flags.** With `use_estimated_contact = false`
  the swing foot's light touch at the end of the step stays below the 20 N
  ground-truth threshold, so every step goes briefly late (the foot is pushed
  down until the force passes 20 N). With the estimated contact the light
  touch is detected and steps end on time.
- **Faster touchdown.** Raising the observer cutoff (15 → 25 to 30 Hz) would
  shorten the force rise by a few ms at the cost of more noise; it needs a
  simulation run to evaluate (it cannot be replayed from the log).
- **Parameter tuning.** The fusion covariances are the paper's Cheetah 3
  values and the model parameters were set by hand. The CSV has every input
  needed for an offline fit of the paper's cost (§V-A).
- **Real robot.** Base pose and twist come from MuJoCo (`Cheat_StateEstimator`);
  a real estimator would need the contact state in turn. The applied torque
  would come from motor current and needs friction and rotor-inertia
  compensation.
- **Touch-sensor leftovers.** `RobotSensor::left/right_touch_sensor`, the reads
  in `Robot_Simulator` and the MJCF `<touch>` sensors are no longer used by
  any code path and can be removed.

## 9. API summary

```cpp
// Construction (biped_walking.cpp)
ContactFusion contact_estimator(robot.model(), {left_foot_frame, right_foot_frame}, dt, 15.0);
contact_estimator.setSoleCorners(0, left_contact_points);   // 3 x 4, foot frame
contact_estimator.setSoleCorners(1, right_contact_points);

// Every control tick, in this order
contact_estimator.computeDisturbanceTorque(q, v, tau_applied);
contact_estimator.fuseContactProbability({s_phi_L, s_phi_R}, {phi, phi});

// Outputs, foot i = 0 (left), 1 (right)
contact_estimator.getContactState(i);        // ŝ
contact_estimator.getContactProbability(i);  // fused P(c)
contact_estimator.getFootWrench(i)(2);       // f̂_z [N]
contact_estimator.getFootHeight(i);          // p_z [m]
contact_estimator.getGroundHeight(i);        // adaptive ground z_g [m]
contact_estimator.getPhaseProbability(i);    // inputs of the last fusion step
contact_estimator.getHeightProbability(i);
contact_estimator.getForceProbability(i);
```

| Parameter | Member / key | Value |
|---|---|---|
| Observer cutoff | constructor `lamda` | 15 Hz |
| Wrench solve damping | `wrench_damping_` | 0.02 |
| f̂_z clamp | `f_z_max_` | 3 m g |
| Phase model σ | `sigma_phase_c_` | 0.1 |
| Height model margin above ground, σ | `muy_zg_`, `sigma_zg_` | 0.01 m, 0.005 m |
| Force model μ, σ | `muy_fc_`, `sigma_fc_` | 20 N, 10 N |
| Fusion covariances | `kQPhase_`, `kRHeight_`, `kRForce_` | 0.998, 0.841, 0.5 |
| Hysteresis | `p_on_`, `p_off_` | 0.5, 0.4 |
| Touchdown confirmation | `touchdown_confirm_time` (YAML) | 0.025 s |
| Earliest touchdown phase | `touchdown_phase_min` (YAML) | 0.7 |
| Late touchdown confirmation / timeout | `late_confirm_time`, `late_touchdown_timeout` (YAML) | 0.005 s, 0 (none) |
| Late descent speed / ramp / max depth | `late_descent_speed`, `late_descent_ramp_time`, `late_max_depth` (YAML) | 0.2 m/s, 0.05 s, 0.02 m |
