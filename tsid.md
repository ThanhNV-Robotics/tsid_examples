# TSID biped control framework

This document explains how the biped demos in this repository control the
robot with TSID (Task Space Inverse Dynamics): the data flow, the QP that is
solved every control step, and the design decisions and fixes that made
standing, CoM swaying, stepping in place and slow walking work.

All gains and weights live in [config/tsid_config.yaml](config/tsid_config.yaml);
the values quoted below are the ones in use when this was written. Treat the
YAML as the source of truth.

---

## 1. Pipeline

One control step runs at 1 kHz (`dt = 0.001` s, same as the MuJoCo timestep):

```
MuJoCo (Robot_Simulator)
  │  getRobotSensorValues()        joint q/dq/tau, IMU, foot touch sensors
  ▼
Cheat_StateEstimator::estimate()   base pose/twist from the free joint (ground truth),
  │                                 contact flags from the touch sensors (> 20 N)
  ▼  RobotState                     world-frame base velocities
utils::robotStateToPinocchio()     q = [p, quat(x,y,z,w), qj], v = [v_B, w_B, dqj]
  ▼
Planners
  MyGaitScheduler                   gait phase phi, leg state (LSt / RSt / DSt)
  CP_Planning::planWalking()        capture point, desired ZMP, CoM pos/vel/acc
  FootPlacement::StepSwingPlanning  footstep + cycloid swing trajectory (pos/vel/acc)
  ▼
tsidTaskParser                      builds tasks/contacts from YAML, contact switching
  ▼
InverseDynamicsFormulationAccForce::computeProblemData() → HQP
solver (eiquadprog-fast)            → dv, f
tsid.getActuatorForces()            → tau → sim.setControl(tau)
```

| Component | File |
|---|---|
| Simulator, viewer, real-time plots, CoM / sphere markers | [sim_interface/Robot_Simulator.h](sim_interface/Robot_Simulator.h) |
| Ground-truth state estimator | [algorithms/cheat_state_estimator.h](algorithms/cheat_state_estimator.h) |
| Gait phase and leg state | [algorithms/my_gait_scheduler.h](algorithms/my_gait_scheduler.h) |
| Capture-point CoM / ZMP planner | [algorithms/CP_Planning.h](algorithms/CP_Planning.h) |
| Footstep and swing trajectory | [algorithms/foot_placement.h](algorithms/foot_placement.h) |
| YAML → TSID tasks, contacts, swing switching | [common/parse_tsid_tasks.h](common/parse_tsid_tasks.h) |
| Shared data types | [common/data_type.h](common/data_type.h) |
| CSV logger | [common/data_logger.h](common/data_logger.h) |
| IK, quintic trajectory, state conversion | [utils/utils.h](utils/utils.h) |
| Demos | [demos/biped_standing.cpp](demos/biped_standing.cpp), [demos/biped_swing_com.cpp](demos/biped_swing_com.cpp) |

---

## 2. The QP

### 2.1 Decision variables

`InverseDynamicsFormulationAccForce` optimizes accelerations and contact
forces; torques are not variables:

$$
x = \begin{bmatrix} \dot v \\ f \end{bmatrix},\qquad
\dot v \in \mathbb{R}^{18}\ (6\ \text{base} + 12\ \text{joints}),\qquad
f \in \mathbb{R}^{12\,n_c}
$$

Each `Contact6d` contributes 4 sole corners × 3D force = 12 variables, so
`f` has 24 entries in double support and 12 in single support.

After solving, torques follow from the actuated rows of the dynamics:

$$
\tau = S\,(M\dot v + h - J_c^\top f)
$$

### 2.2 Level 0: hard constraints

| Constraint | Form |
|---|---|
| Floating-base dynamics (6 unactuated rows) | $M_u \dot v + h_u = J_{c,u}^\top f$ |
| Rigid contact, per planted foot (6 rows) | $J_c \dot v + \dot J_c v = -K_p e - K_d \dot e$ |
| Friction pyramid, per sole corner | $\lvert f\cdot t_{1,2} \rvert \le \mu\, (f\cdot n)$ |
| Normal force bounds, per foot (sum over corners) | $f_{min} \le \sum_i n\cdot f_i \le f_{max}$ |
| Torque limits | $-\tau_{max} \le S(M\dot v + h - J_c^\top f) \le \tau_{max}$ |

Notes, verified in the TSID source:

- The pyramid uses $\mu$ directly (not $\mu/\sqrt2$), so it slightly
  over-approximates the true cone at its diagonals.
- `f_min` / `f_max` bound the **total** normal force of a foot, not each corner.
  Each corner is kept non-pulling by the pyramid rows.
- Because every corner force must push, the centre of pressure automatically
  stays inside the support polygon of the 4 corners: the ZMP condition is
  built in, no separate ZMP task is needed.

### 2.3 Level 1: weighted cost

$$
\begin{aligned}
J(x) =\; & w_{com}\,\lVert S_{xy}(J_{com}\dot v + \dot J_{com} v - \ddot c^*)\rVert^2 \\
 + & w_{base}\,\lVert S_{rp}(J_{base}\dot v + \dot J_{base} v - a^*_{base})\rVert^2 \\
 + & w_{swing}\,\lVert J_{sw}\dot v + \dot J_{sw} v - a^*_{sw}\rVert^2 \quad\text{(only while a foot is in the air)}\\
 + & w_{post}\,\lVert \dot v_j - \ddot q^*_j\rVert^2 \\
 + & w_{reg}\sum_{feet}\lVert W(G f - f_{ref})\rVert^2 + \varepsilon\lVert x\rVert^2
\end{aligned}
$$

Every motion task uses a PD law with feedforward,
$a^* = a_{ref} + K_p(x_{ref}-x) + K_d(\dot x_{ref}-\dot x)$:

| Task | Frame / mask | Kp | Weight |
|---|---|---|---|
| CoM | x, y only (`mask [1,1,0]`) | 30 | 5.5 |
| Base orientation | `link0_torso`, roll + pitch only | 10 | 1.0 |
| Swing foot (`TaskSE3Equality`) | `*_ankle_pitch_link`, full 6D | 100 | 10 |
| Joint posture | 12 joints | 30–120 (per joint) | 0.001 |
| Force regularization | foot wrench, $W=\mathrm{diag}(1,1,10^{-3},2,2,2)$, $f_{ref}=0$ | — | 1e-3 |

Kd defaults to $2\sqrt{K_p}$ (critical damping) everywhere.

The solver sees $\min_x \tfrac12 x^\top H x + g^\top x$ with
$H = \sum_i 2w_i A_i^\top A_i + \varepsilon I$, $g = -\sum_i 2w_i A_i^\top b_i$.

### 2.4 Priorities: weights, not null spaces

TSID does **not** use null-space projection. `eiquadprog` solves only two
levels: hard constraints (level 0) and one weighted cost (level 1).
Level-1 tasks compete through their weights, so for example the CoM task
(5.5) dominates posture (0.001). Strict multi-level priorities would need a
cascade of QPs or a hybrid scheme (OpenLoong-style null-space kinematics +
dynamics QP), which this project does not use.

Cost: about 0.18 ms per step on average (problem build + solve), worst
case about 0.8 ms, so well inside the 1 ms period.

---

## 3. Planning

### 3.1 Capture-point CoM / ZMP planner

LIPM with $w = \sqrt{g/z_c}$, capture point $\xi = c + \dot c / w$:

$$
\dot \xi = w(\xi - p),\qquad \dot c = w(\xi - c),\qquad \ddot c = w^2(c - p)
$$

At the start of each gait cycle the planner picks a CP target $\xi_d$ (one
step forward, on the next stance side) and holds the ZMP constant so that the
CP arrives exactly at $\xi_d$ at the end of the cycle:

$$
p = \frac{\xi_d - b\,\xi_0}{1-b},\qquad b = e^{wT}
$$

Because $b \approx 67$ for $T = 1$ s, $p \approx \xi_0$: during cycle $k$ the
ZMP stays where the CP **started**, i.e. on the side targeted in cycle $k-1$.
That is why the swing leg is delayed by one cycle (section 4.4).

Forward progress: the CP target advances `step_length = vx * t_swing` per
cycle, so the CoM moves at the commanded speed.

### 3.2 Footstep and swing trajectory

`FootPlacement` lands the swing foot relative to the **stance foot**:

$$
p_{land} = p_{stance} + R_z(\psi_{cmd})\begin{bmatrix} v_x T \\ \pm w_{stance}\end{bmatrix},
\qquad z_{land} = z_{stance} + z_{offset}
$$

with $\psi_{cmd}$ the joystick heading and `stepLength = vx * tSwing`
clamped to `max_step_length`. A zero command steps in place.

The swing path is a cycloid with a cosine height bump, $\phi\in[0,1]$,
$\dot\phi = 1/T$:

$$
c(\phi) = \frac{2\pi\phi - \sin 2\pi\phi}{2\pi},\qquad
z(\phi) = z_0 + \tfrac12 h(1-\cos 2\pi\phi) + \Delta z\, c(\phi)
$$

Position, velocity and acceleration are computed analytically (checked
against finite differences to ~1e-9) and fed to the swing task.

---

## 4. Essential strategies and fixes

These are the things that made the difference; most were found by logging
and comparing planned vs. measured signals.

### 4.1 Model and state

- **Free-flyer model.** The URDF has no root joint, so the robot must be
  loaded with `pinocchio::JointModelFreeFlyer()`. Without it TSID thinks the
  torso is bolted to the world and only computes the torques to move the
  legs in the air; the robot collapses.
- **Conventions between MuJoCo and Pinocchio.** MuJoCo quaternions are
  `(w,x,y,z)`, Pinocchio's are `(x,y,z,w)`. MuJoCo's free-joint linear
  velocity is in the world frame and its angular velocity in the body frame;
  Pinocchio's free-flyer velocity is fully local.
  `utils::robotStateToPinocchio()` does the conversion.
- **One model.** The standing IK uses the `RobotWrapper` model (free-flyer)
  instead of re-parsing the URDF; `computeIK_Leg` holds the base fixed and
  solves only the joint columns.

### 4.2 Gains and units

- **TSID gains are acceleration gains** ($\ddot q^* = K_p e + K_d \dot e$),
  not Nm/rad joint PD gains. Kd is set to $2\sqrt{K_p}$ (critical damping);
  the YAML joint `kd` values from the real robot would give a damping ratio of
  about 0.05.
- **Posture weight must be small (0.001).** At 0.1 the posture task resisted
  the lateral CoM shift more than the CoM task pushed for it (hip/ankle roll
  need ~0.14 rad to move the CoM 10 cm).

### 4.3 Contacts

- **Sole corners from the URDF.** The 4 contact points are the bottom face of
  each foot's collision box, read with urdfdom (URDF link frames equal
  Pinocchio body frames). Values: x −0.05…0.13, y ±0.03, z −0.04 in the ankle
  frame. `Contact6d` requires exactly 4 points; the parser checks it.
- **Contact normal** `(0,0,1)` is in the foot frame; it defines the friction
  pyramid axes.
- **Grounded start.** `computeGroundedConfiguration()` places the base so the
  lowest sole corner is 1 mm above the floor, using the real corner positions
  (a tilted foot would otherwise start inside the ground).

### 4.4 CoM / ZMP planning

- **CoM feedforward is essential.** The CoM task receives the planned CoM
  velocity and acceleration (`getCoMvelRef()`, `getCoMaccRef()`,
  $\ddot c = w^2(c-p)$). With zero reference derivatives the Kd term brakes
  every planned motion and the real CoM lags far behind the plan (±0.025 m
  instead of ±0.13 m). With feedforward the error is a few mm.
- **Seed the capture point.** `setInitCom()` sets the CP to the CoM (robot at
  rest) and recomputes $w$. Otherwise the CP stays at world (0,0) and drags
  the CoM reference there at the start of walking.
- **Lateral width = measured stance width.** `cp_planner.wd_hip` is set to the
  actual distance between the planted feet when walking starts, so the ZMP
  targets lie on the feet.
- **One-cycle swing delay.** `planWalking()` publishes
  `leg_state_swing_ = oppositeLeg(previous leg state)` and `phi_swing`. In
  steady state that is the same leg as the current state; the first cycle has
  no swing (DSt), which gives the CoM time to move onto the first stance foot
  before anything lifts. With this, the planned ZMP is inside the stance sole
  100 % of every swing.
- `gait_scheduler.firstleg = RSt`: the first cycle shifts the CoM toward the
  left foot, so the right foot swings first.

### 4.5 Swing foot and contact switching

Handled by `tsidTaskParser::startSwing()` / `endSwing()` /
`setSwingReference()`:

- **Lift-off:** `removeRigidContact(name, contact_transition_time)` ramps the
  foot's max normal force to zero, then removes the contact; the swing task
  (`TaskSE3Equality`) is added at the same time.
- **Touchdown:** remove the swing task, set the contact reference to the
  foot's **actual** landed pose, add the contact again.
- **Restore `f_max` on touchdown (TSID quirk).** TSID lowers a contact's max
  normal force during the removal ramp and never restores it. Re-adding the
  same contact object gave the landed foot a ~0 N limit; it could not carry
  the robot once the other foot lifted, and the robot fell on every second
  step. `endSwing()` now calls `setMaxNormalForce(f_max)`.
- **Early touchdown from the touch sensor.** After mid-swing ($\phi > 0.5$)
  the foot is planted as soon as its touch sensor reports contact, instead of
  pushing it further toward the (possibly below-ground) target until the phase
  ends, which tilted the body.
- **Swing references in world-aligned axes.** `TaskSE3Equality` in local
  mode expects the reference velocity/acceleration in world-aligned axes
  (it applies `actInv` itself); `FootPlacement` outputs exactly that.
- **Nominal foot yaw.** Each swing targets the foot's nominal orientation
  (flat, standing yaw captured when walking starts), not its liftoff
  orientation. Otherwise yaw slip of the stance foot accumulates step after
  step (feet ended up at +15° / −22° and the robot fell).
- **`z_offset` is a touchdown stretch relative to the ground** (stance foot
  height), not to the liftoff height; in the planned footstep chain a landed
  foot is recorded at ground height, so a negative offset cannot make the
  steps sink.

### 4.6 Base

- **Base orientation task on roll and pitch only.** Reduced base tilt about
  3× during CoM sway. Holding yaw as well made things worse while the feet
  could still slip.

### 4.7 Simulation

- **Foot friction.** The foot collision boxes use
  `friction="2.0 0.05 0.001" condim="6"` (MuJoCo default: `1 0.005 0.0001`,
  `condim 3`, i.e. no torsional friction). This cut the stance-foot yaw slip
  from 2.5–9° to about 1–1.7° per step. TSID still assumes `mu = 0.8`, which
  keeps the planned forces conservative.

---

## 5. Debugging tools

- **CSV log** (`DataLogger`, OpenLoong-style named items): joint commands and
  measurements, torques, CoM ref/measured, base and foot roll/pitch/yaw,
  swing leg/phase/reference, planned CoM/ZMP, landing points. Plot with
  [scripts/plot_joint_control.py](scripts/plot_joint_control.py) or pandas.
- **Viewer markers:** yellow sphere = measured CoM; red dot + line = desired
  ZMP; green sphere = swing-foot target; blue spheres = liftoff / landing.
- **Real-time plots:** left/right leg torques, CP planner signals, swing foot
  height.
- **Plan-only mode:** `enable_swing = false` keeps both feet planted and
  chains the planned footsteps (`openLoopFootsteps`), to check the planner
  signals without moving the robot.

---

## 6. Status and known limitations

- Standing, CoM swaying and stepping in place are stable; walking at
  `vx = 0.05` m/s works for about 24 s (~1.2 m) before it falls.
- **The CP planner aims at fixed world targets** (`yBias ± wd_hip/2`), not at
  the measured stance foot. Small landing errors make the feet drift sideways
  (~0.5 mm/step) until the ZMP leaves the sole. Next step: base the CP/ZMP
  targets on the actual stance foot.
- **Base yaw is free** and swings ±4° per step; combined with drift it grows
  until the robot twists over.
- **Torque limits come from the URDF `effort` (1000 Nm)**, not the MuJoCo
  `ctrlrange` (±16…48 Nm); add `tau_max` to `actuation_bounds_task` in the YAML
  to make TSID plan with the real limits.
- The state estimator reads ground truth from MuJoCo; a real estimator must
  replace `Cheat_StateEstimator` before hardware use.
