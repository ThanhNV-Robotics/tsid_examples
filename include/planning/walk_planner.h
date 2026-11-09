#pragma once
#include "Eigen/Dense"

#include "common/data_type.h"
#include "planning/lip_task_planner.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <algorithm>
#include <string>
#include <tsid/robots/robot-wrapper.hpp>

namespace biped_controllers {
    // phases of WalkPlanner: INIT / STOP / REST Tswing long (REST repeats until the next reset);
    // a STEP is a double support (double_support_time) then the swing, which ends at the touchdown
    //   INIT: after reset(), both feet down, support frame on the left ankle; the CP is driven to
    //         the left ankle (shifted by the CoM bias) so the right foot can lift off, no swing.
    //         Then the first STEP (right foot swings, no double support), or STOP at once if
    //         requestStop() was called
    //   STEP: one step: double support (in_ds_, if double_support_time > 0: both feet loaded, ZMP
    //         in the box around both soles, CP driven toward the stance foot), then the trailing
    //         foot swings on a cycloid to the (CP adapted) foothold. Ends at the touchdown (Tswing,
    //         or touchdown() early / late): the landed foot is the stance foot, the next STEP. After
    //         requestStop() one closing step (the feet end side by side), then STOP
    //   STOP: both feet down, CP held in the middle of the feet (shifted by the CoM bias), the
    //         swing foot reference at rest. Then REST
    //   REST: double support, CoM at rest over the middle of the feet; repeats until reset()
    // reset() -> INIT -> STEP -> ... -> (requestStop) closing STEP -> STOP -> REST (repeats)
    // the double support / swing split is inside STEP (in_ds_): isSwinging(), inDoubleSupport()
    enum class WalkPhase { INIT, STEP, STOP, REST }; // top level-state

    // Walking planner: CoM (LIPTaskPlanner) + swing foot references, both planned in the support
    // frame and returned in the world frame (TSID tasks).
    //
    // Support frame: on the ankle of the stance foot (left foot in double support), yaw only (x
    // forward along the foot, z up), taken from the forward kinematics of the estimated state once
    // per phase and then fixed until the next switch, so the references do not shake with the
    // estimate. At a switch the LIP state is re-expressed in the new frame.
    //
    // Phases (step timing: see below), as the earlier planner of tsid_examples:
    //   init phase (both feet down): CP to the left ankle shifted by the CoM bias (com_bias_: the
    //   CoM keeps its place over the feet, e.g. its x ahead of the ankle), no swing
    //   steps, right foot first: the swing foot moves from liftoff to the foothold (step_length
    //   ahead, the stance width aside of the stance ankle) on a cycloid (zero velocity at both
    //   ends); the CP is driven to the end-of-step CP of the periodic gait, with the receding-horizon
    //   ZMP (CP feedback) clamped to the stance sole
    //   last step (the step after requestStop()): a closing step, the swing foot lands beside the
    //   stance foot (foothold x = 0, the stance width aside: the feet end side by side, parallel
    //   to the pelvis); the end-of-step CP is the middle between the stance ankle and the foothold
    //   (shifted by the CoM bias), so the CP is between the feet when the swing foot lands. (The
    //   periodic gait ends a step with the CP on the new stance ankle: moving it back to the
    //   middle then needs the ZMP outside that foot.)
    //   stop (after the last step): both feet down, CP held in the middle of the feet (shifted by
    //   the CoM bias)
    //   rest: double support, CoM at rest over the middle of the feet, until the next reset()
    // The LIP height is the CoM height and the stance width the ankle distance of the pose at
    // reset() (the robot walks at that height, on that width).
    //
    // Step timing (tsid_examples walk_planner.cpp, event based, Stage 3b): a step starts with a
    // double support (both feet loaded, ZMP in the box around both soles, CP driven to the start of
    // the single support of the periodic gait), then the trailing foot lifts off and swings. The
    // swing ends at the touchdown: time based at Tswing; event based (setWaitForTouchdown(true),
    // Bledt et al. ICRA 2018, Sec. VI) at touchdown(), early or late. The touchdown ends the step
    // at once: new support frame on the landed foot, the plan pulled toward the measured CoM / CP,
    // the next double support. Without touchdown() at Tswing the step is late: the ZMP stays on the
    // stance sole (the CP diverges) and the swing foot keeps descending below the foothold.
    //
    // Heading (setYawRateCommand()): at each support switch the heading of the coming step advances
    // by the yaw rate times one step period (double support + swing), at most max_turn_per_step from
    // the new stance foot. The nominal step (step_length, +-stance width), the CP targets and the
    // foothold limits are defined in the heading axes and rotated by the heading error (heading -
    // stance foot yaw) into the support frame; the swing foot turns to the heading on the cycloid
    // of the swing. The torso yaw reference is the mean of the two feet yaw references.
    class WalkPlanner
    {
        public:
            // robot: free-flyer robot model, owned by the caller (shared with the other modules, this
            // planner has its own pinocchio::Data); yaml: section "LIPPlanner" (LIP + step settings);
            // foot frames: the ankle frames of the support frames. Throws std::runtime_error if the
            // file, a value or a frame is invalid
            WalkPlanner(tsid::robots::RobotWrapper &robot, const std::string &CONFIG_YAML_PATH,
                        const std::string &left_foot_frame, const std::string &right_foot_frame);

            // restart from the estimated state q (both feet on the ground, at rest): support frame
            // on the left ankle, CoM at rest, LIP height = CoM height of q, init phase next
            void reset(const Eigen::VectorXd &q);

            // stop walking: the current phase ends as planned (a swing foot lands), then one last
            // step that brings the CP to the middle of the feet, then the stop phase and rest (from
            // the init phase: the stop phase at once). Cleared by reset()
            void requestStop() { this->stop_requested_ = true; }

            // true: a step ends at touchdown() (late state if the step reaches Tswing first); false:
            // at Tswing (time based, default)
            void setWaitForTouchdown(bool wait) { this->wait_touchdown_ = wait; }
            // the swing foot of this step has landed (event based, confirmed contact): the step ends
            // in the next planWalking(), early or late. Ignored outside the swing
            void touchdown() { this->touchdown_ = true; }

            // yaw rate command [rad/s] (z up, left turn positive), used from the next step on
            void setYawRateCommand(double wz) { this->yaw_rate_cmd_ = wz; }

            // forward speed [m/s] -> step length (clamped to [0, max_step_length]), used from the
            // next step on
            void setVelocityCommand(double vx);

            // main function run in controller update(): one planning cycle with the estimated state
            // q, v (CoM and CoM velocity of the robot: CP feedback; forward kinematics of the feet
            // only at a switch). Real-time (no allocation)
            void planWalking(const Eigen::VectorXd &q, const Eigen::VectorXd &v);

            // ---- references, world frame (valid after planWalking()) ----
            const Vector3d &getRefCoMPosition() const { return this->com_pos_; }
            const Vector3d &getRefCoMVelocity() const { return this->com_vel_; }
            const Vector3d &getRefCoMAcceleration() const { return this->com_acc_; }
            // swing foot ankle; in the init phase the right foot standing (velocity 0)
            const Vector3d &getRefSwingFootPosition() const { return this->swing_foot_pos_; }
            const Vector3d &getRefSwingFootVelocity() const { return this->swing_foot_vel_; }
            const Vector3d &getRefSwingFootAcceleration() const { return this->swing_foot_acc_; }
            // swing foot orientation: flat, turning from its liftoff yaw to the heading of the step;
            // its yaw rate and acceleration [rad/s, rad/s^2] (about the world z axis)
            const Eigen::Matrix3d &getRefSwingFootRotation() const { return this->swing_rot_; }
            double getRefSwingFootYawRate() const { return this->swing_wz_l_; }
            double getRefSwingFootYawAcc() const { return this->swing_dwz_l_; }
            // torso yaw reference: mean of the stance foot and the swing (other) foot yaw references
            // (continuous across a touchdown); its rate and acceleration
            double getRefBaseYaw() const { return this->base_yaw_; }
            double getRefBaseYawRate() const { return 0.5 * this->swing_wz_l_; }
            double getRefBaseYawAcc() const { return 0.5 * this->swing_dwz_l_; }
            double getHeading() const { return this->heading_; } // world yaw of the current step
            Vector2d getRefZMP() const;  // world (x, y)
            Vector2d getRefCP() const;   // world (x, y)
            Vector2d getMeasuredCP() const; // world (x, y): CP of the robot, c + dc / w
            // swing foot landing point (ankle), world; adapted to the CP during the step
            Vector3d getFoothold() const { return this->oMs_.act(this->foothold_); }

            // ---- walking state ----
            WalkPhase getWalkPhase() const { return this->phase_; }
            // a foot is in the air (single support of a STEP): getSwingFoot() is that foot (0 left,
            // 1 right); otherwise both feet are on the ground (double support, init, stop, rest)
            bool isSwinging() const { return this->phase_ == WalkPhase::STEP && !this->in_ds_; }
            bool inDoubleSupport() const { return this->phase_ == WalkPhase::STEP && this->in_ds_; }
            int getSwingFoot() const { return this->left_stance_ ? 1 : 0; }
            bool isLeftStance() const { return this->left_stance_; }        // true: right foot swings
            bool isInitWalkingDone() const { return this->phase_ != WalkPhase::INIT; }
            // swing phase in [0, 1] (time / Tswing, 1 while late); 0 when no foot swings
            double getPhase() const { return this->isSwinging() ? std::min(this->phase_time_ / this->T_, 1.0) : 0.0; }
            // late touchdown: the step reached Tswing without touchdown() (setWaitForTouchdown(true))
            bool isLate() const { return this->late_; }
            // steps completed since reset() (touchdowns; the closing step counts too)
            int getStepCount() const { return this->step_count_; }
            double getLateTime() const { return this->late_time_; }         // [s] in the late state
            const pinocchio::SE3 &getSupportFrame() const { return this->oMs_; } // world <- support
            double getDt() const { return this->lip_.getDt(); }
            double getTswing() const { return this->T_; }                    // step (swing) duration [s]

        private:
            // support frame on the ankle of the given foot, from the forward kinematics of q (yaw only)
            pinocchio::SE3 supportFrame(const Eigen::VectorXd &q, bool left);
            // CP target, ZMP limits (support frame) and remaining time of the current (sub)phase
            double phaseTarget(Vector2d &cp_target, Vector2d &zmp_min, Vector2d &zmp_max) const;
            // INIT / STOP / REST ended (Tswing): next phase; INIT -> first step (no double support)
            void endOfPhase(const Eigen::VectorXd &q, const Eigen::VectorXd &v);
            // touchdown (or Tswing, time based): the landed foot is the stance foot, the next step
            // starts with its double support (or the stop phase after the last step)
            void endStep(const Eigen::VectorXd &q, const Eigen::VectorXd &v);
            // after a step: the landed foot is the stance foot, new support frame, LIP state into it
            // and pulled toward the measured one, liftoff position of the other foot
            void switchSupport(const Eigen::VectorXd &q, const Eigen::VectorXd &v, bool first_step);
            // CoM and CP of the robot (q, v) in the support frame, given to the LIP planner
            void measureCP(const Eigen::VectorXd &q, const Eigen::VectorXd &v);
            // CP-based foothold adaptation (tsid_examples walk_planner.cpp, Stage 3): the foothold
            // from which the next step starts like the planned one, for the predicted end-of-step
            // CP; cp_target: the CP target of this step
            void adaptFoothold(const Vector2d &cp_target);
            // both feet stay down: CP target in the middle of the feet, swing foot reference at rest
            void startStop();
            // nominal foothold of the step: step_h_ rotated by the heading error into the support frame
            void setNominalFoothold();
            // swing foot reference at phase phi (support frame), from swing_start_pos_ to foothold_
            void computeSwingTrajectory(double phi);
            // late state: the swing foot reference descends below the foothold (support frame)
            void computeLateDescent();
            // references of this cycle from the support frame to the world frame
            void toWorld();

            tsid::robots::RobotWrapper &robot_; // robot model, owned by the caller
            pinocchio::Data data_;               // forward kinematics of this planner
            pinocchio::FrameIndex foot_id_[2];   // ankle frames [left, right]

            LIPTaskPlanner lip_;                 // CoM plan, in the support frame

            double T_{1.0}; // swing time (= init / stop phase duration)
            double t_ds_{0.0};                   // [s] double support at the start of each step (0: none)
            double max_step_length_{0.3};
            double hip_width_{0.27};             // stance width [m]: ankle distance measured at reset()
            double sole_x_min_{-0.04};           // ZMP limits in single support [m], stance ankle frame
            double sole_x_max_{0.12};            // (defaults of tsid_examples walk_planner.h)
            double sole_y_half_{0.02};
            double step_height_{0.05};           // swing foot peak height [m] (as tsid_examples)
            // [m] the swing trajectory ends this far below the foothold: the foot meets the ground
            // before the step end with a downward velocity (an impact the contact estimator sees)
            double swing_landing_depth_{0.0};
            double step_length_{0.0};            // from setVelocityCommand
            double ankle_height_{0.0};           // ankle above the ground [m] (reset): LIP zc -> ankle frame
            // late touchdown: descent of the swing foot below the foothold (as the earlier
            // planner), speed ramped up from 0 (no velocity jump), at most max depth
            double late_descent_speed_{0.2};     // [m/s]
            double late_descent_ramp_time_{0.05}; // [s], <= 0: no ramp
            double late_max_depth_{0.02};        // [m], <= 0: no limit
            // CP-based foothold adaptation (defaults of tsid_examples)
            bool foothold_adaptation_{true};
            double foothold_max_width_change_{0.05}; // [m] lateral range around the nominal foothold
            double foothold_max_rate_{0.5};          // [m/s] largest foothold change per second
            double foothold_freeze_phase_{0.8};      // the foothold no longer changes after this phase
            double max_turn_per_step_{0.2};          // [rad] largest heading change of a step

            WalkPhase phase_ = WalkPhase::INIT;  // phase of this cycle
            bool stop_requested_ = false;        // requestStop(): last step from the next phase end on
            bool last_step_ = false;             // this step ends walking: lands beside the stance foot, CP between the feet
            Vector2d stop_target_ = Vector2d::Zero(); // CP target of STOP / REST: middle of the feet (support frame)
            Vector2d com_bias_ = Vector2d::Zero();    // CoM - middle of the feet at reset (support frame axes)
            bool left_stance_ = true;            // true: left foot stands, right foot swings
            double phase_time_ = 0.0;            // [s] in the current phase (STEP: in the double support / swing)
            bool in_ds_ = false;                 // STEP: in its double support
            bool wait_touchdown_ = false;        // setWaitForTouchdown(): a step ends at touchdown()
            bool touchdown_ = false;             // touchdown() in this swing
            int step_count_ = 0;                 // getStepCount()
            bool late_ = false;                  // the step is past Tswing without touchdown
            double late_time_ = 0.0;             // [s] in the late state
            Vector2d cp_meas_ = Vector2d::Zero(); // measured CP of this cycle, support frame

            // heading
            double yaw_rate_cmd_ = 0.0;          // [rad/s] setYawRateCommand()
            double heading_ = 0.0;               // [rad] world yaw of the current step (walking direction)
            double heading_err_ = 0.0;           // [rad] heading - stance foot yaw: rotation of the step
            Vector2d step_h_ = Vector2d::Zero(); // nominal step in the heading axes (step_length, +-stance width)
            // yaw of the swing foot (both feet down: the other foot) relative to the support frame:
            // at liftoff, reference of this cycle and its rate / acceleration (cycloid, as the position)
            double swing_yaw_start_l_ = 0.0, swing_yaw_l_ = 0.0, swing_wz_l_ = 0.0, swing_dwz_l_ = 0.0;
            double base_yaw_ = 0.0;              // [rad] torso yaw reference (world)
            Eigen::Matrix3d swing_rot_ = Eigen::Matrix3d::Identity(); // swing foot orientation (world)

            pinocchio::SE3 oMs_ = pinocchio::SE3::Identity(); // support frame (world <- support)

            // swing foot ankle (support frame); _l_ = local, the world copies are below
            Vector3d swing_start_pos_ = Vector3d::Zero(); // at liftoff (FK at the switch): start of the cycloid
            Vector3d foot_nominal_ = Vector3d::Zero();    // nominal foothold (step_length, +-stance width, 0): CP targets
            Vector3d foothold_ = Vector3d::Zero();        // landing: nominal, then adapted to the CP; end of the cycloid,
                                                          // z = 0: stance ankle height (CP target: periodic gait, planWalking)
            Vector3d swing_pos_l_ = Vector3d::Zero();     // reference of this cycle: cycloid + z bump at phase phi
            Vector3d swing_vel_l_ = Vector3d::Zero();     // d/dt = (d/dphi) / T (phi = t / T), 0 at both ends
            Vector3d swing_acc_l_ = Vector3d::Zero();     // d2/dt2 = (d2/dphi2) / T^2

            // references (world frame) for TSID, from the support frame oMs_ in toWorld(): positions
            // with the full transform, velocities / accelerations rotated only (frame fixed in a step)
            Vector3d com_pos_ = Vector3d::Zero();         // LIP (x, y), zc above the ground
            Vector3d com_vel_ = Vector3d::Zero();         // LIP w (cp - com), z = 0
            Vector3d com_acc_ = Vector3d::Zero();         // LIP w^2 (com - zmp), z = 0
            Vector3d swing_foot_pos_ = Vector3d::Zero();  // swing_pos_l_ in the world
            Vector3d swing_foot_vel_ = Vector3d::Zero();  // swing_vel_l_ rotated
            Vector3d swing_foot_acc_ = Vector3d::Zero();  // swing_acc_l_ rotated
    };
}
