# Event-based walking control: implementation plan

Plan for making the biped walk event-based: each step ends on a touchdown
detected by `ContactFusion` (see [contact_fusion.md](contact_fusion.md)), not on
a timer, and the CoM / capture-point (CP) plan stays consistent with whatever
the actual step timing turns out to be. The work is split into four stages,
each tested before the next one starts. This file is the plan; the
implemented result will be documented separately once the stages are done.

---

## 1. Starting point

### 1.1 What works

With the current configuration the robot walks stably on flat ground (60 s,
55 steps, max base roll 0.7°; another run walked 463 s). The planner already
has the event machinery:

- `planWalking()` takes the contact flags as input; `biped_walking.cpp` passes
  the fusion's estimate `ŝ` (`use_estimated_contact = true`).
- Early touchdown: the step ends when the swing foot's contact is confirmed for
  `touchdown_confirm_time` (25 ms) after `touchdown_phase_min` (φ = 0.7).
- Late touchdown: at φ = 1 without contact the support leg is kept and the swing
  foot is lowered (ramped descent) until contact or `late_touchdown_timeout`.

### 1.2 Why it is not event-based yet

On flat ground the swing foot ends every step resting on the floor but
unloaded (0 N at φ = 1, fused probability 0.39 to 0.40, below `p_on` = 0.5).
With `late_touchdown_timeout: 0.005` every step goes late and ends 5 ms later
by the timeout: all 55 steps of the 60 s test. The walk is a 1.005 s timer.

Every attempt to let contact events end the steps destabilized the walk:

| Change (flat ground) | Steps ended by | Result |
|---|---|---|
| late timeout 0.005 s (current) | timeout, all steps | stable |
| late timeout 0, descent 1 m/s | late → contact event (12 to 18 ms late) | lateral error grows 12 → 48 → 68 → 312 mm, falls at step 5 |
| late timeout 0, descent 0.2 m/s with ramp | late → contact event (19 to 27 ms late) | CoM 90 mm behind the plan at a touchdown, falls at step 3 |
| `z_offset` −5 mm (timeout 0.005 s) | early event at φ ≈ 0.92 | falls in the first step |
| `z_offset` −10 mm (timeout 0.005 s) | early events at φ ≈ 0.70 to 0.87 | falls at step 2 |

### 1.3 Root cause: the walk planner assumes a fixed step duration

Per step, `enterStance()` re-anchors the planning frame to the stance ankle and
stores the **planned** CP `ξ₀`. Then for the whole step
([walk_planner.cpp](../src/planning/walk_planner.cpp), `planWalking()`):

```
b     = e^(ω·Tswing)
ξ_d   = (s·b/(b−1),  ±w_h·b/(b+1))          periodic end-of-step CP (s = step length, w_h = stance width)
p_zmp = clamp((ξ_d − b·ξ₀) / (1 − b))       constant ZMP that reaches ξ_d after exactly Tswing
ξ, c  integrated open loop from p_zmp        (computeCP, computeCoM)
next foothold = (s, ±w_h)                     fixed
```

Three properties make any timing change a disturbance:

1. **Fixed duration.** `p_zmp` is chosen once, for a step of exactly `Tswing`.
   A step that ends early leaves the CP short of `ξ_d`; a late step lets it
   overshoot. The error is frozen into the next step's `ξ₀`.
2. **Open loop.** The plan integrates its own CP and never reads the robot.
   In the failing runs the measured CoM lagged the plan by 48 to 90 mm
   sideways and the planner did not react.
3. **Fixed footholds.** The comment in the code says "the next foothold absorbs"
   the CP error, but `next_foot_pos_ = (step_length, side·wd_hip)` does not
   depend on the CP. The only correction left is the ZMP, with ±20 mm of
   lateral authority (`sole_y_half`), which normal steps already use up to
   ±15 mm. It saturated before every fall.

## 2. Plan

### Stage 1: receding-horizon ZMP

Recompute the ZMP every tick from the current CP and the **remaining** step
time instead of once per step:

```
T_rem = Tswing · (1 − φ)
p_zmp = clamp( (ξ_d − e^(ω·T_rem)·ξ) / (1 − e^(ω·T_rem)) )
```

- CP errors are spread over the rest of the step instead of being frozen at
  the step start; for an undisturbed step the result equals the current
  constant ZMP.
- The formula is ill-conditioned as `T_rem → 0` (the denominator goes to 0):
  below `T_rem < zmp_hold_time` (about 30 ms) keep the last ZMP.
- Still uses the planned CP `ξ`; no behaviour change expected on the stable
  baseline.

Changes: `planWalking()` (the two `zmp_*_d_` lines), new YAML key
`zmp_hold_time`.

Acceptance: the 60 s flat baseline is unchanged (stable, max roll < 1°).

### Stage 2: close the loop with the measured CP

Use the measured CP, `ξ_meas = c + ċ/ω` (CoM and CoM velocity of the robot,
re-expressed in the planning frame), in the Stage 1 formula. The planner
becomes a DCM feedback controller: when the robot lags, the ZMP command
reacts.

- Blend instead of switching outright: `ξ = ξ_plan + k_ξ·(ξ_meas − ξ_plan)`,
  `k_ξ` = 0.3 to 0.5 to start (the measured CP is noisier than the plan).
- The CoM reference for TSID keeps being integrated from the commanded ZMP, but
  at every touchdown event `enterStance()` resets the planned CoM and CP to the
  measured ones (blended the same way), so plan and robot cannot drift apart
  over many steps.
- `planWalking()` already receives the robot and its `pinocchio::Data`
  (`robot_wrapper.com(data)`, `com_vel(data)`), and `reanchor()` /
  `pointToWorld()` already handle the frame conversion.

Changes: `planWalking()`, `enterStance()`, new YAML key `cp_feedback_gain`.
Log the measured and planned CP (CSV) for tuning.

Acceptance: flat baseline stable; the lateral CoM error at touchdown is
smaller than without feedback.

### Stage 3: CP-based foothold adaptation

Place the next foot where the nominal periodic gait resumes from the CP the
robot will actually have at touchdown. Predicted end-of-step CP (constant ZMP
over the remaining time):

```
ξ_end = p_zmp + e^(ω·T_rem) · (ξ − p_zmp)
```

For the periodic gait with the ZMP at the stance ankle, a step starts with the
CP at `ξ₀ = ξ_d / b` relative to its stance foot. The next step's nominal
start CP, relative to the next stance foot, is therefore

```
ξ₀_next = ( s/(b−1),  −side·w_h/(b+1) )
```

and the foothold that makes the next step start nominal is

```
p_next = ξ_end − ξ₀_next
```

For an undisturbed step `ξ_end = ξ_d`, which gives `p_next = (s, side·w_h)`,
the current fixed foothold.

- Clamp `p_next` to kinematic limits: step length (`max_step_length`), a
  lateral range around the nominal width (no leg crossing, no over-wide
  step), and a maximum change per tick so the swing target does not jump.
- Freeze the foothold near the end of the swing (for example φ > 0.8) so the
  landing point does not move under the descending foot.
- `FootPlacement` takes the landing point from `getNextFootW()`, so the swing
  trajectory follows without changes there.

Changes: `planWalking()` (`next_foot_pos_`), new YAML keys for the foothold
limits.

Acceptance: flat baseline stable; then a lateral push (drag the robot in the
viewer, or a scripted force) is recovered by a wider step instead of ZMP
saturation.

### Results of Stages 1 to 3, and why a double-support phase is needed

Stages 1 to 3 are implemented. Flat walking with the current configuration
stays stable (Stage 2 halves the lateral CoM error at touchdown, Stage 3 keeps
the footholds nominal while the ZMP is not at its limit), but the configurations
that let contact events end the steps still fall. The traces show why: in this
gait (1 s single support, instant switch between feet, ZMP at the stance
ankle) the capture point arrives at the new foot exactly at the step end.

With ω = √(g / z_c) = 4.24 s⁻¹ (z_c = 0.545 m), `b = e^(ωT)` ≈ 69, the planned CP
at the end of the step is 4 mm short of the new ankle and moves toward it at
about 1 m/s (ω times the stance width). Every millisecond the old stance phase
is extended pushes the CP about 1 mm further; a touchdown 19 ms late put the
CP on the outer edge of the new stance sole, where neither the ZMP (±20 mm)
nor the next foothold (on the other side; no crossing steps) can bring it back.
Shorter steps (0.5 s, 0.4 s) were tried and fall even with ground-truth
contact: the gait's tuning (swing task gain, contact transition time) is
matched to 1 s steps.

### Stage 3b: double-support phase

Each step starts with a double-support phase (DS) at the touchdown event,
during which both feet carry the robot and the CP is moved onto the new stance
foot; the trailing foot lifts off at its end:

```
touchdown event ──► DS (T_ds, both feet in contact) ──► liftoff ──► SS (T_ss, swing) ──► touchdown event ...
```

**Why it solves the timing problem.** During DS the ZMP can be anywhere in the
support polygon of both feet (about 30 cm laterally instead of ±2 cm), so
whatever state a touchdown arrives in, early or late, DS can bring the CP back
to the nominal start of the next single support. And the nominal CP at the end
of single support is far from the new foot:

| T_ss / T_ds | CP at touchdown, before the new ankle | CP speed | late-touchdown tolerance before DS |
|---|---|---|---|
| 1.0 s / 0 (current) | 4 mm | 1.07 m/s | about none |
| 0.8 s / 0.2 s | 76 mm | 0.76 m/s | about 70 ms |
| 0.7 s / 0.3 s | 95 mm | 0.69 m/s | about 110 ms |

Start with T_ss = 0.8 s, T_ds = 0.2 s: the step period stays 1 s, so the swing
gains and contact transition tuned for this gait still fit.

**Nominal (periodic) CP targets.** ZMP constant at the stance ankle during SS
and at the midpoint between the feet during DS; for a step of length s and
stance width w (next foot at `(s, side·w)`), with `E_ss = e^(ω T_ss)` and
`E_ds = e^(ω T_ds)`, the CP at the start of SS relative to the stance foot is

```
ξ₀ = ( s (1 + E_ds) / (2 (E_ds E_ss − 1)),   side · w (1 + E_ds) / (2 (1 + E_ds E_ss)) )
ξ_ss_end = E_ss · ξ₀          CP at touchdown (end of SS), relative to the stance foot
```

For `T_ds = 0` this reduces to the current targets (`s/(b−1)`, `side·w/(b+1)`).

**Planner (`WalkPlanner`):**

- Step state: DS then SS. The touchdown event (`enterStance()`, unchanged
  confirmation and late logic) starts the DS of the new stance foot; the
  planning frame is re-anchored to the new foot as now.
- DS: receding-horizon ZMP toward `ξ₀` (relative to the new stance foot) over the
  remaining DS time, clamped to the support region of both feet (the band
  between the two ankles, sole half-width wide); a new liftoff event at the end
  of DS.
- SS: receding-horizon ZMP toward `ξ_ss_end` over the remaining SS time,
  clamped to the stance sole (Stage 1 and 2, with the new target).
- `getSwingPhase()` runs over SS only (0 during DS); new `inDoubleSupport()` and
  `liftoffEvent()`; the measured-CP feedback and plan reset (Stage 2) apply in
  both phases.
- Stage 3 foothold: place the next foot so that the predicted CP at touchdown
  is at the nominal `ξ_ss_end` relative to it.

**Demo (`biped_walking.cpp`):** split the contact switch. Touchdown event →
`endSwing()` of the landing foot only (both feet in contact). Liftoff event →
`startSwing()` of the trailing foot. The contact fusion gets both feet
scheduled in contact during DS.

**Config:** `t_swing: 0.8` (single support), new `double_support_time: 0.2`.

**Tests:** the same matrix (current config, timeout 0, `z_offset` −5 mm), first
with ground-truth contact to validate the DS gait, then with the contact
estimate; then the box.

### Stage 4: event-based step timing

With Stages 1 to 3 the plan is consistent with any touchdown time, so the
events can drive the steps:

- **Touchdown event** (early or late): `enterStance()` starts the next step
  from the measured CP (Stage 2 reset); the receding-horizon ZMP and the
  adapted foothold take care of the rest.
- **Late state** (φ > 1, no contact): `T_rem = 0`, so the CP cannot be steered
  through timing. Put the ZMP as close to the CP as the stance sole allows
  (this slows the divergence) and keep updating the next foothold from the
  CP, so the descending foot lands where it can catch the robot.
- Set `late_touchdown_timeout` back to 0 (wait for the touchdown) and retest
  the configurations of section 1.2 that failed.
- Optional later: choose each step's duration and foothold together from the
  CP state with a small QP (step timing adaptation, Khadiv et al. 2016 / 2020).

Acceptance: flat ground stable with every step ended by a contact event (no
timeouts), then the 1.5 cm box step-up and step-down.

## 3. Test protocol

Every stage is tested with headless runs (no windows):

```
TSID_HEADLESS=1 TSID_SIM_TIME=60 ./build/biped_walking
```

- `TSID_HEADLESS=1` creates all windows hidden
  ([Robot_Simulator.cpp](../src/sim/Robot_Simulator.cpp)); `TSID_SIM_TIME`
  stops the demo at that simulated time.
- The evaluation helper used so far (summary of how each step ended, survival,
  max roll) lives outside the repo; as part of Stage 1 it moves to
  `scripts/` so the runs are reproducible.

Metrics per run:

| Metric | Source |
|---|---|
| survival time, QP failure | console log |
| how each step ended: early event / late → event / late → timeout | CSV (`swing_leg`, `swing_phase`, `late_touchdown`) and console log |
| max base roll and pitch while walking | CSV `base_rpy` |
| lateral CoM error at touchdown (plan vs measured) | CSV `plan_com`, `com_meas` |
| time with the ZMP at the sole limit | CSV `plan_zmp` vs stance foot |
| contact estimation accuracy | console log |

Test matrix, run after each stage:

1. Flat ground, current config (timeout 0.005 s): must stay stable.
2. Flat ground, `late_touchdown_timeout: 0`: the configuration that fell.
3. Flat ground, `z_offset` −5 mm: early events at φ ≈ 0.9 that fell.
4. Box step-up and step-down (`step_box` in the scene), from Stage 4.

## 4. Files and parameters

| File | Stage | Change |
|---|---|---|
| `src/planning/walk_planner.cpp`, `include/planning/walk_planner.h` | 1 to 4 | ZMP recomputation, measured CP, foothold adaptation, late-state ZMP |
| `config/tsid_config.yaml` | 1 to 3 | new keys under `walk_planner: foot_placement:` |
| `demos/biped_walking.cpp` | 2 | log measured / planned CP |
| `scripts/` | 1 | headless run and evaluation helper |
| `doc/` | after 4 | document the implemented controller |

New parameters (initial values, to be tuned):

| Key | Stage | Initial value | Meaning |
|---|---|---|---|
| `zmp_hold_time` | 1 | 0.03 s | below this remaining time the ZMP is held |
| `cp_feedback_gain` | 2 | 0.3 | blend of measured into planned CP (0 = open loop, 1 = measured) |
| `foothold_max_width_change` | 3 | 0.05 m | lateral foothold range around the nominal width |
| `foothold_freeze_phase` | 3 | 0.8 | the foothold no longer changes after this step phase |

## 5. Risks

- **Measured-CP noise and interaction with TSID.** The CoM velocity is noisier
  than the plan, and TSID tracks the CoM reference with its own gains; full
  feedback can oscillate. Mitigation: the blend gain of Stage 2, raised
  gradually.
- **Foothold adaptation without limits** can produce crossing or overstretched
  steps from a single bad estimate. Mitigation: kinematic clamps and the
  freeze phase of Stage 3.
- **Small lateral ZMP range.** With a 6 cm wide sole the ZMP alone can never
  absorb much; Stage 3 is expected to be the main stability gain, Stages 1
  and 2 mostly prepare it.
- **Late state.** Single support is held longer while the CP diverges; the
  foothold adaptation must be active during the late state for the landing
  to catch the robot.
