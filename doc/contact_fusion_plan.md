# Contact model fusion: implementation plan

Plan for implementing the probabilistic contact detector of
Bledt, Wensing, Ingersoll, Kim, *"Contact Model Fusion for Event-Based
Locomotion in Unstructured Terrains"*, ICRA 2018, on the biped in this repo.

**Goal: the controller must not use a foot contact sensor.** The real robot
has none. The MuJoCo `lf-touch` / `rf-touch` sensors exist only in simulation.
At the end of this work, contact comes only from proprioception:

- joint encoders (q, q̇),
- joint torques (motor current × torque constant on the robot; MuJoCo
  `actuator_force × gear` in simulation, already in `actuator_state.torquej`),
- the IMU,
- the gait schedule.

The simulator still knows the true contact state. That is used **only for
evaluation**: logging, accuracy and delay. It never feeds the controller, and
it does not come from a sensor the robot would need to carry.

The paper has three parts. Each becomes one part of the implementation:

| Paper | What it does | Here |
|---|---|---|
| §III | Discrete-time generalized-momentum (GM) disturbance observer → per-foot force estimate without a force sensor | `MomentumObserver` (new) |
| §IV | Kalman-filter fusion of 3 contact probabilities: gait phase (prior), foot height, estimated foot force | `ContactFusion` (`include/estimation/contact_fusion.h`, currently a stub) |
| §VI | Event-based FSM per leg: Swing / Contact / Early / Late | changes in `WalkPlanner` + `biped_walking.cpp` |

---

## 0. Where it plugs into the current pipeline

Today ([tsid.md](tsid.md) §1):

```
sensors ─► Cheat_StateEstimator ─► RobotState.contact_flags = touch sensor > 20 N
                                         │
                                         ▼
                         WalkPlanner::planWalking()  (touchdown = swing_lifted_ &&
                                                       swing_contact && phi >= phi_td_min_)
```

Target:

```
sensors ─► Cheat_StateEstimator ─► RobotState (base + joints from ground truth)
               │
               ├─► MomentumObserver::update(q, v, tau)    ─► f̂_L, f̂_R   (§III)
               │
               └─► ContactFusion::update(phase prior from WalkPlanner (previous tick),
                                         foot heights p_z, f̂_z)        ─► P_L(c), P_R(c), ŝ_L, ŝ_R
                                         │
                                         ▼
                   contact_flags = ŝ   (the only contact source)
                                         │
                                         ▼
                   WalkPlanner::planWalking()  ─► event-based FSM (§VI)

  (sim only, evaluation) Robot_Simulator::getGroundTruthContacts() ─► logger
```

**Ground truth without the touch sensor**: read MuJoCo's contact list
(`mjData::contact` + `mj_contactForce`) and sum the normal force of every
contact on each foot geom. This is a simulator query, not a robot sensor, so
the `<touch>` sensors and their sites can be removed from the MJCF. It is
exposed as a separate simulator call so it cannot end up in `RobotSensor` by
accident.

Ordering: the phase prior comes from the planner state of the **previous**
tick (the planner itself consumes the fused flags this tick). There is a
1-tick (1 ms) lag in the prior, and it removes the circular dependency.

---

## 1. Step 1: discrete-time momentum observer (paper §III)

### 1.1 Equations (paper conventions)

Equation of motion, with `v` the Pinocchio velocity (nv = 6 + 12):

```
M(q) v̇ + C(q,v) v + g(q) = Sᵀ τ + τ_d,    τ_d = Σ_i J_iᵀ w_i
```

with `p = M v` and C the Christoffel factorization (Ṁ = C + Cᵀ). The
discrete-time observer, eq. (10):

```
γ = exp(-λ Δt)                 λ = cutoff [rad/s] (paper: 15 Hz → λ = 2π·15)
β = (1 - γ) / (γ Δt)
x_k   = β p_k + Sᵀ τ_k + C_kᵀ v_k - g_k
y_k   = γ y_{k-1} + (1 - γ) x_k          ← first-order LPF  (1-γ)/(1-γ z⁻¹)
τ̂_d,k = β p_k - y_k
```

No joint acceleration is needed.

For the Fig. 3 comparison, also implement the classical continuous-time
observer, eq. (3), discretized with the same LPF but with λ in place of β:
`τ̂_d = λ p - LPF_λ(λ p + Sᵀτ + Cᵀv - g)`. Select it with a flag
(`observer: discrete | continuous`).

### 1.2 Per-foot wrench (paper §III-C)

Paper: `f̂_i = (S_i J_iᵀ)⁻¹ S_i τ̂_d`. The biped has 6 joints per leg and
6D contacts, so `S_i J_iᵀ` is **6×6** and maps to the full sole wrench:

- `J_i`: 6×nv frame Jacobian of the contact frame (`left/right_ankle_pitch_link`)
  in `LOCAL_WORLD_ALIGNED`, shifted to the sole point (`sole_z` from the
  `contact_tasks` YAML), so that `f̂_z` is the vertical force in the world frame.
- `S_i`: picks the leg's 6 columns of `v` (get them from
  `model.joints[id].idx_v()`, do not hard-code).
- Solve with `colPivHouseholderQr()` or damped LS. `S_i J_iᵀ` becomes
  ill-conditioned with a straight knee. Log its condition number.

Alternative to keep in mind (not the paper's method): the 6 floating-base rows
of `τ̂_d` also see both feet. A least-squares solve over all 18 rows
`[J_Lᵀ J_Rᵀ] [w_L; w_R] = τ̂_d` uses that extra information. Implement it only
if the per-leg estimate turns out too noisy.

### 1.3 Pinocchio calls (per tick, 1 kHz)

```cpp
pinocchio::crba(model, data, q);                      // upper triangle only →
data.M.triangularView<Eigen::StrictlyLower>() =
    data.M.transpose().triangularView<Eigen::StrictlyLower>();
pinocchio::computeCoriolisMatrix(model, data, q, v);  // data.C, satisfies Ṁ = C + Cᵀ
pinocchio::computeGeneralizedGravity(model, data, q); // data.g
pinocchio::computeJointJacobians(model, data, q);  + updateFramePlacements
pinocchio::getFrameJacobian(model, data, foot_id, LOCAL_WORLD_ALIGNED, J);
```

Use a **separate `pinocchio::Data`** owned by the observer so it does not race
with the `data` that TSID and the planners use.

### 1.4 Practical issues

- **Which τ**: the torque that was actually applied. In simulation that is
  `RobotSensor.actuator_state.torquej`: `Robot_Simulator.cpp` fills it with
  `actuator_force × gear`, which is the applied torque after clamping. On the
  robot it is the measured motor current × torque constant × gear ratio. That
  measurement includes gearbox friction and does not include motor rotor
  inertia, so it needs the same compensation as in the next point. Do not use
  the commanded `tau`: it differs from the applied torque whenever the
  actuator saturates.
- **Model mismatch between MJCF and URDF**: MuJoCo joint `armature`,
  `damping` and `frictionloss` are not in the Pinocchio model. During a fast
  swing they show up as a fake disturbance (this is the error the paper's
  Fig. 3 shows). Compensate with values read from `mjModel` (`dof_armature`
  → add to diag(M); `dof_damping` → subtract `D q̇` from `Sᵀτ`). Make this a
  YAML switch so its effect can be measured.
- **Initialization**: at the first tick set
  `y_0 = β p_0 + Sᵀτ_0 + C_0ᵀ v_0 - g_0` minus the quasi-static disturbance
  `(g_0 - Sᵀτ_0)`, so `τ̂_d` starts at the standing-support value instead of
  a large transient. Simpler alternative: start the observer during `T_stand`
  and ignore its output until it has converged (a few `1/λ`).
- **Velocity source**: `v` comes from `Cheat_StateEstimator` (ground truth),
  so the base terms are exact in simulation. On hardware this would be the
  weak point. Out of scope here.

### 1.5 Validation (before any fusion)

1. Standing: `f̂_z,L + f̂_z,R ≈ m g` (robot mass from `pinocchio::computeTotalMass`).
2. Compare `f̂_z` with the simulator's ground-truth contact force (§0) over a
   walking run. Log both.
3. Swing foot: `f̂_z ≈ 0`. Compute the RMS over swing phases for the discrete
   and continuous observers (reproduces Fig. 3). Expect discrete < continuous.

---

## 2. Step 2: contact probability models + Kalman fusion (paper §IV)

### 2.1 State, prediction, correction

State `x = [P_L(c), P_R(c)]ᵀ`, N = 2. The paper uses `A = 0`, `B = I`, `H = [I; I]`:

```
prediction:  x⁻ = u                  Σ⁻ = Σ_w
correction:  z = [z_height; z_force],   Σ_v = blkdiag(Σ_v1, Σ_v2)
             K = Σ⁻ Hᵀ (H Σ⁻ Hᵀ + Σ_v)⁻¹,  x = x⁻ + K (z - H x⁻)
```

All matrices are diagonal and `A = 0`, so this reduces exactly to an
independent, inverse-variance weighted average **per foot**:

```
P(c) = (u/σ_w² + z1/σ_v1² + z2/σ_v2²) / (1/σ_w² + 1/σ_v1² + 1/σ_v2²)
```

Implement this scalar form. It is cheaper and clearer. Add a unit test that
compares it with the matrix KF form on random inputs. Clamp the result to [0, 1].

Paper tuned values: `Σ_w = 0.998 I`, `Σ_v1 = 0.841 I`, `Σ_v2 = 0.930 I`
(roughly equal weights). Start from these.

### 2.2 Prior: gait phase, u = P(c | s_φ, φ)

```
P = ½ ( s_φ [ erf((φ-μ_c0)/(σ_c0√2)) + erf((μ_c1-φ)/(σ_c1√2)) ]
      + s̄_φ [ 2 + erf((μ_c̄0-φ)/(σ_c̄0√2)) + erf((φ-μ_c̄1)/(σ_c̄1√2)) ] )
```

Mapping to the biped (`WalkPlanner` getters):

| Planner state | Left foot | Right foot |
|---|---|---|
| `getStanceLeg() == LSt` | `s_φ=1`, φ = `getSwingPhase()` | `s_φ=0`, φ = `getSwingPhase()` |
| `getStanceLeg() == RSt` | `s_φ=0`, φ = step phase | `s_φ=1`, φ = step phase |
| `DSt` (standing / init shift) | `s_φ=1`, φ = 0.5 | `s_φ=1`, φ = 0.5 |

Each foot stands for one step and swings for the next, so the subphase of the
stance foot is the step phase. Note that the planner can end a step early on
touchdown, so φ is not strictly linear in time. This is fine, because the
prior only needs the scheduled phase.

Table I values are **variances**: μ = {0, 1}, σ² = 0.05 → σ ≈ 0.224, for both
stance and swing.

### 2.3 Measurement 1: foot height, P(c | p_z)

```
P(c | p_z) = ½ [1 + erf((μ_zg - p_z) / (σ_zg √2))]
```

- `p_z`: world height of the sole (ankle frame translation + rotated `sole_z`).
- `μ_zg`: expected ground height. Start with 0 (flat MuJoCo floor). Then
  adapt it, as the paper suggests: set it to the sole height recorded at the
  last confirmed touchdown, which makes the model terrain-blind on steps.
- `σ_zg²`: terrain roughness. The paper uses 0.1 m², which is very wide for a
  5 cm step height (`stepHeight: 0.05`). Expect to tighten it to about
  (0.01 to 0.02 m)². Tune it (step 5).

### 2.4 Measurement 2: estimated force, P(c | f_z)

```
P(c | f_z) = ½ [1 + erf((f̂_z - μ_fc) / (σ_fc √2))]
```

Paper: μ_fc = 40 N, σ_fc² = 25 N² (on a ~45 kg quadruped). Start at
μ_fc ≈ 30 N (the current touch threshold is 20 N), σ_fc² ≈ 25 N², then scale
with this robot's weight.

### 2.5 Binary decision

Eq. (26) with hysteresis, as the paper describes:
`ŝ: 0→1 when P > P_on`, `1→0 when P < P_off`, with e.g. `P_on = 0.6` and
`P_off = 0.4`.

### 2.6 Class layout

`{include,src}/estimation/momentum_observer.{h,cpp}`

```cpp
class MomentumObserver {
public:
  MomentumObserver(const pinocchio::Model& model, const YAML::Node& cfg,
                   const std::vector<std::string>& foot_frames,
                   const std::vector<double>& sole_z);
  void reset(const VectorXd& q, const VectorXd& v, const VectorXd& tau);
  void update(const VectorXd& q, const VectorXd& v, const VectorXd& tau);
  const VectorXd& disturbance() const;   // τ̂_d (nv)
  Vector6d footWrench(int i) const;      // [f; m], world-aligned, at the sole
private:
  pinocchio::Data data_;   // own Data, see 1.3
  double gamma_, beta_; VectorXd y_;
  ...
};
```

`{include,src}/estimation/contact_fusion.{h,cpp}` (replaces the stub, which also has a syntax
error: `class ContactFusion:`)

```cpp
struct ContactFusionParams { /* μ/σ of the phase, height and force models, Σ_w, Σ_v1, Σ_v2, P_on, P_off */ };

class ContactFusion {
public:
  explicit ContactFusion(const YAML::Node& cfg);
  // per foot: scheduled state, subphase, sole height, estimated vertical force
  void update(const std::array<bool,2>& s_phi, const std::array<double,2>& phi,
              const std::array<double,2>& p_z, const std::array<double,2>& f_z);
  double probability(int foot) const;         // fused P(c)
  bool   contact(int foot) const;             // ŝ after hysteresis
  // per-model probabilities, for logging / tuning
  double pPhase(int foot) const, pHeight(int foot) const, pForce(int foot) const;
  void   setGroundHeight(int foot, double z);  // adaptive μ_zg
};
```

`ContactFusion` does not depend on Pinocchio or MuJoCo (plain numbers in,
probabilities out), so it can be unit-tested and replayed on logged data
offline.

### 2.7 YAML (`config/tsid_config.yaml`)

```yaml
contact_fusion:
  # No contact-source switch: fusion is the only source. During bring-up,
  # use `shadow_mode: true` to compute and log the fusion while the planner
  # still uses the simulator's ground-truth flags.
  shadow_mode: false
  observer:
    type: discrete              # discrete | continuous
    cutoff_hz: 15.0
    compensate_mj_armature: true
    compensate_mj_damping: true
  phase_model:  {mu_c0: 0.0, mu_c1: 1.0, var_c0: 0.05, var_c1: 0.05,
                 mu_s0: 0.0, mu_s1: 1.0, var_s0: 0.05, var_s1: 0.05}
  height_model: {mu_zg: 0.0, var_zg: 0.0004, adaptive: true}
  force_model:  {mu_fc: 30.0, var_fc: 25.0}
  covariance:   {phase: 0.998, height: 0.841, force: 0.930}
  threshold:    {p_on: 0.6, p_off: 0.4}
  fsm:          {t_delay: 0.01}  # [s] contact confirmation delay (§VI)
```

---

## 3. Step 3: integrate into `biped_walking.cpp` (detection only)

1. Construct `MomentumObserver` and `ContactFusion` after `robot` and `tsid`.
   Call `observer.reset()` at t = 0.
2. In the control loop, after `state_estimator.estimate()`:
   - `observer.update(q, v, tau_applied)`
   - build the per-foot prior from `walk_planner.getStanceLeg()` and
     `getSwingPhase()` (previous tick, see §0)
   - `fusion.update(...)`
3. Remove contact from the sensor/estimator path:
   - delete `left_touch_sensor` / `right_touch_sensor` from `RobotSensor`;
   - delete the touch-flag code (and `contact_force_level`) from
     `Cheat_StateEstimator`. The estimator then only provides the base and
     joint state;
   - `planWalking` takes the contact flags as an argument (from
     `ContactFusion`) instead of reading `state_estimator.getState().contact_flags`;
   - ground truth for logging comes from `sim.getGroundTruthContacts()` (§0).
4. `DataLogger` items: `f_hat_z` (L, R), `f_gt_z` (L, R), `p_z` (L, R),
   `P_phase`, `P_height`, `P_force`, `P_fused` (L, R), `s_hat`, `s_gt`,
   `s_phi` (L, R), `observer_cond`.
5. Realtime plot: `P_fused` vs `s_gt` per foot (the paper's Fig. 7 / 9 view).
6. Metrics printed at exit: classification accuracy `mean(ŝ == s_gt)`, and
   mean / max touchdown and liftoff delay (time from the `s_gt` edge to the
   `ŝ` edge).

Acceptance for this step:
- the MJCF has no `<touch>` sensor and the code has no reference to one
  (`grep -rn touch` finds only comments / ground-truth evaluation);
- walking on flat ground is as stable as the current touch-sensor version;
- accuracy > 98 %;
- touchdown delay below about 10 ms.

---

## 4. Step 4: event-based gait FSM (paper §VI)

Per leg, driven by `(ŝ, s_φ)`:

```
            ŝ=1 (s_φ=0)                    t ≥ t_c + t_delay
  Swing ───────────────► Early ─────────────────────────► Contact
    ▲  │                                                    │
    │  │ scheduled swing over, ŝ=0                          │ s_φ=0 (scheduled liftoff)
    │  └──────────────────► Late ──── ŝ=1 ──────────────────┤
    └───────────────────────────────────────────────────────┘
```

Mapping to the current code:

| FSM | Current behaviour | Change |
|---|---|---|
| **Early** (contact before scheduled end) | `WalkPlanner` already accepts a touchdown if `phi_CoM >= phi_td_min_`, and switches stance at once | On `ŝ=1` during swing: **freeze the swing reference** at the measured foot position, with zero velocity and acceleration (paper: "hold current foot position"). Only after `ŝ` stays 1 for `t_delay`: raise the step event → `task_parser.endSwing()` adds the rigid contact. If `ŝ` drops back to 0 before then (bounce), resume the swing. `phi_td_min_` can then be lowered or removed. |
| **Late** (scheduled end, no contact) | `step_end` forces the stance change and the contact is added mid-air ("stomp") | At φ ≥ 1 without `ŝ`: keep the foot in swing and keep lowering the swing target at a constant speed (e.g. 0.1 m/s, capped at a max reach below the planned height). Hold the planner phase at 1. Switch stance only on `ŝ=1`. Add a safety timeout. |
| **Contact** | — | Leaves Contact only on the scheduled liftoff, the existing path through `startSwing()` with `contact_transition_time`. |

Implementation location: a small `LegContactFSM` struct (two instances)
inside `ContactFusion` or next to it. `WalkPlanner` asks it
`touchdownConfirmed(leg)` instead of reading raw flags. `FootPlacement` gets
`freeze()` and `extendDown()` hooks for the swing reference.

Test scenarios (MuJoCo scene variants):

1. Flat ground: the FSM should stay in Swing/Contact (no Early/Late).
2. A box of 2 to 4 cm under the next foothold → Early.
3. A step down / hole → Late.

For each, compare foot bounce and base attitude with and without the FSM
(paper Fig. 11).

---

## 5. Step 5: parameter tuning (paper §V-A)

The paper fits all model parameters offline (`fmincon`) with

```
J(χ) = (s - ŝ)ᵀ W_c (s - ŝ) + (s - P(c))ᵀ W_n (s - P(c)),   W_c ≫ W_n
```

on logs from flat ground, rough terrain and a step.

Here: `scripts/tune_contact_fusion.py` reads the `DataLogger` output (inputs
`s_phi, phi, p_z, f_hat_z` and label `s_gt`), re-implements the scalar fusion
in numpy, optimizes χ = {μ, σ for each model, the 3 variances, P_on/P_off}
with `scipy.optimize.minimize` (bounded, e.g. L-BFGS-B or Nelder-Mead,
since ŝ is non-smooth), and writes the result as a YAML snippet. This works
because `ContactFusion` is pure (§2.6), so offline replay matches online
behaviour exactly.

---

## 6. File checklist

| File | Action |
|---|---|
| `{include,src}/estimation/momentum_observer.{h,cpp}` | new |
| `{include,src}/estimation/contact_fusion.{h,cpp}` | rewrite the stub |
| `{include,src}/estimation/cheat_state_estimator.{h,cpp}` | remove touch-sensor contact flags |
| `include/common/data_type.h` | remove `left/right_touch_sensor` from `RobotSensor`; `RobotState::contact_flags` filled from `ContactFusion` |
| `{include,src}/sim/Robot_Simulator.{h,cpp}` | remove touch-sensor reads; add `getGroundTruthContacts()` (evaluation only) |
| `models/biped/mjcf/*biped*.xml` | remove `<touch>` sensors and their `*-tc` sites |
| `{include,src}/planning/walk_planner.{h,cpp}` | prior getters per foot; event-based touchdown (step 4) |
| `{include,src}/planning/foot_placement.{h,cpp}` | swing freeze / extend-down hooks (step 4) |
| `demos/biped_walking.cpp` | wiring, logging, metrics |
| `config/tsid_config.yaml` | `contact_fusion:` block |
| `scripts/tune_contact_fusion.py` | new (step 5) |
| `doc/tsid.md` | add a pipeline section once step 3 works |

CMake needs no change: `src/**/*.cpp` is globbed.

## 7. Order and milestones

1. Add `getGroundTruthContacts()` and check that it matches the touch sensors.
   This is the last time the touch sensors are used.
2. Observer + logging only, with the planner on ground-truth flags →
   validate §1.5.
3. Fusion in shadow mode (computed and logged, not used) → accuracy/delay
   numbers.
4. Planner on fused flags; **remove the touch sensors** from MJCF,
   `RobotSensor`, the simulator and the estimator (§3 step 3).
5. Offline tuning on logs from steps 3 and 4.
6. Event-based FSM + terrain scenes.

## 8. Risks / open questions

- **MJCF ↔ URDF dynamics mismatch** (armature, damping, friction, link
  inertias): the biggest source of fake force in swing. Measure it in step 1
  before tuning anything downstream.
- **Applied torque**: torque limits (`tau_max`) and actuator saturation in
  MuJoCo can make the commanded τ differ from the applied τ. Use the measured
  torque if possible.
- **Near-singular legs** (straight knee at stance) degrade `(S_i J_iᵀ)⁻¹`.
- **Phase-prior circularity**: the prior comes from the planner, and the
  planner's phase depends on past contact decisions. Only the previous tick's
  phase is used, which keeps this causal, but watch for lock-in, e.g. a missed
  touchdown that keeps the swing prior low. The Late branch of the FSM is the
  mitigation.
- **Base state on the real robot**: `Cheat_StateEstimator` reads the base
  pose and twist from the simulator. The observer needs the base velocity
  (`p = M v` includes the base). A real estimator (IMU + leg kinematics) in
  turn needs to know which feet are in contact, so on hardware the fused
  contact also feeds the base estimator. Plan for that loop, but it is a
  separate task.
- **No fallback sensor**: with the touch sensor gone, a fusion failure has
  nothing to fall back on. The FSM timeouts (Late branch) and the hysteresis
  are the safety net. Also add sanity checks: in double support,
  `f̂_z,L + f̂_z,R ≈ m g`; and never declare both feet out of contact while
  the planner is in `DSt`.
- **Paper scale**: Cheetah 3 values (40 N, 0.1 m² ground variance) do not
  transfer directly to this biped. Treat Table I only as a starting shape.
