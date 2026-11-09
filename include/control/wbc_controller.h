#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <tsid/robots/robot-wrapper.hpp>

#include "common/data_type.h"
#include "control/controller_params.h"
#include "control/qp_tsid.h"
#include "estimation/contact_fusion.h"
#include "estimation/state_estimator.h"
#include "planning/walk_planner.h"

namespace biped_controllers
{

// Motion commands (humanoid_mpc_msgs/MotionCommand in the ROS 2 project)
enum class MotionCommand { HOMING = 1, INIT_WALKING = 2, WALKING = 3 };

// Base state of the simulator, for params.use_ground_truth: world position, orientation, world
// linear velocity, base-frame angular velocity (the MuJoCo free joint convention)
struct GroundTruth
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

// Whole-body controller: state estimation, contact fusion, LIP walk planner and TSID. ROS-free port
// of biped_controllers/WBCController of the ROS 2 project, same logic (see the FSM in its README):
//   motion mode STAND <-> WALK, walk phases of the planner, contact FSM of the swing foot.
// The caller owns the loop:
//   WBCController wbc(params);
//   wbc.activate(sensor);                 // from a robot standing on both feet
//   each control period wbc.getDt():
//     wbc.update(t, sensor);              // then send getJointPosCmd / VelCmd / TorqueCmd
// Joint vectors are in Pinocchio order. Commands take effect in the next update(). The WALKING
// messages of the ROS 2 project (sent while the key is held) are replaced by a number of steps.
class WBCController
{
public:
  // loads the model and builds the modules; throws std::runtime_error if a file / value is invalid
  explicit WBCController(const ControllerParams &params);

  // start from the current state: estimators, TSID and every reference; joint targets of commands
  // 1 / 2 (IK once). Throws if the IK does not converge. ground_truth: see update()
  void activate(const RobotSensor &sensor, const GroundTruth *ground_truth = nullptr);

  // one control cycle. ground_truth: the base state, used if params.use_ground_truth. false = no
  // new command this cycle (invalid sensor reading or QP failed), the last one stays
  bool update(double time, const RobotSensor &sensor, const GroundTruth *ground_truth = nullptr);

  // command 1 homing / 2 init walking: both ignored (warned) while walking or in a transition
  void sendCommand(MotionCommand command);
  // command 3: walk num_steps full steps, then one closing step (the feet end side by side), stop
  // and rest. Needs the init walking pose. 0 = walk until stopWalking()
  void walk(int num_steps);
  // finish the step, closing step, stop and rest
  void stopWalking() { this->stop_command_ = true; }
  // turn command in [-1, 1] (left positive), times max_yaw_rate. Not 0: turning in place
  void setTurnCommand(double wz);

  bool isWalking() const { return this->mode_ == Mode::WALK; }
  bool transitionRunning(double time) const { return time - this->transition_start_time_ < kTransitionDuration; }

  // contact FSM of the swing foot, see SwingState below
  enum class SwingState { SWING, EARLY, LATE, CONTACT };
  SwingState getSwingState() const { return this->swing_state_; }

  double getDt() const { return this->dt_; }
  tsid::robots::RobotWrapper &robotModel() { return *this->robot_pin_wrapper_; }
  const ControllerParams &getParams() const { return this->params_; }
  const Eigen::VectorXd &getJointPosCmd() const { return this->q_cmd_; }
  const Eigen::VectorXd &getJointVelCmd() const { return this->v_cmd_; }
  const Eigen::VectorXd &getTorqueCmd() const { return this->tau_cmd_; }
  const Eigen::VectorXd &getDriveKp() const { return this->drive_kp_; } // gains of the PVT drive
  const Eigen::VectorXd &getDriveKd() const { return this->drive_kd_; }
  const Eigen::VectorXd &getState() const { return this->q_; }          // estimated q (nq)
  const Eigen::VectorXd &getVelocity() const { return this->v_; }       // estimated v (nv)
  const Eigen::VectorXd &getComRef() const { return this->com_ref_; }   // CoM reference (3)
  const WalkPlanner &getWalkPlanner() const { return *this->walk_planner_; }
  const ContactFusion &getContactFusion() const { return *this->contact_fusion_; }
  const StateEstimator &getStateEstimator() const { return *this->state_estimator_; }

private:
  // constructor: robot model, QP_TSID + WalkPlanner, StateEstimator + ContactFusion
  void setupRobotModel ();
  void setupWbc ();
  void setupStateEstimate ();
  void AllocateBuffers ();
  // drive kp / kd of every joint from the gains yaml (section `robot`: both ankle joints take the
  // sum of the two ankle motors' gains); missing -> 0 (warned)
  void loadDriveGains ();

  // activate(): estimators and TSID from the current state
  void resetModules ();
  // activate(): joint targets of commands 1 / 2 and the CoM of each target pose relative to the
  // left foot; every reference held at the current state
  void setupPostureTargets ();

  // update(): sensor_ -> estimated state q_, v_ and contact_probability_ (for the next cycle)
  void estimate ();
  // base state of q_ / v_ from the ground truth of this cycle (joints stay measured)
  void applyGroundTruth ();
  // a new motion command starts its transition or walking
  void handleMotionCommand (double time);
  // quintic from the current references to the joint target and to the CoM of that pose, placed on
  // the left foot where it is now
  void startTransition (const Eigen::VectorXd &qj_target, const Eigen::Vector3d &com_in_foot, double time);
  void startWalking (double time);
  // joint command q_cmd_ / v_cmd_ / tau_cmd_ of this cycle; false = QP failed, the last one stays
  bool computeCommand (double time);
  bool computeWalkCommand (double time);

  double dt_ = 0.0;           // control period of all modules [s] (LIPPlanner sampling_frequency)
  ControllerParams params_;

  // Robot model, shared by every module (each has its own pinocchio::Data): declared before them,
  // members are destroyed in reverse order
  std::unique_ptr<tsid::robots::RobotWrapper> robot_pin_wrapper_;
  std::unique_ptr<QP_TSID> qp_tsid_controller_;      // tsid qp module
  std::unique_ptr<WalkPlanner> walk_planner_;        // walk planner module (LIP CoM + swing foot)
  std::unique_ptr<StateEstimator> state_estimator_;  // state estimator module
  std::unique_ptr<ContactFusion> contact_fusion_;    // contact fusion module

  RobotSensor sensor_;                    // joint q / dq / tau + IMU of this cycle (estimator input)
  const GroundTruth *ground_truth_ = nullptr; // of this cycle
  Eigen::VectorXd q_;                     // configuration [base pos 3, quat 4 (x y z w), joints] (nq)
  Eigen::VectorXd v_;                     // velocity [base twist 6 (base frame), joints] (nv)
  Eigen::VectorXd contact_probability_;   // [left, right] in [0, 1]
  std::vector<bool> scheduled_contact_;   // [left, right] planned stance (contact fusion input)
  std::vector<double> contact_phase_;     // [left, right] subphase in [0, 1] of that plan
  // use_time_base false: the contact estimator saw the swing foot off the ground in this step
  bool liftoff_seen_ = false;
  // contact FSM of the swing foot (Bledt et al. ICRA 2018, Sec. VI, use_time_base false):
  // SWING follows the swing trajectory; EARLY: contact detected before the step ends, the foot is
  // held where it touched down until the contact is confirmed; LATE: the step reached Tswing without
  // contact, the planner waits and the foot keeps descending until a contact (or
  // late_touchdown_timeout); CONTACT: the foot is in TSID contact and the planner was told
  SwingState swing_state_ = SwingState::SWING;
  double early_start_time_ = 0.0;         // [s] update time of the early contact (t0 of the paper)
  pinocchio::SE3 hold_pose_;              // EARLY: swing foot reference, the touchdown pose (world)
  int prev_swing_foot_ = -1;              // swing foot of the last cycle, -1 = none
  Eigen::VectorXd dv_;                    // accelerations solved by TSID (nv)
  Eigen::VectorXd tau_;                   // joint torques solved by TSID (na)
  Eigen::VectorXd q_cmd_;                 // joint command to send: position (na)
  Eigen::VectorXd v_cmd_;                 // velocity (na)
  Eigen::VectorXd tau_cmd_;               // feedforward torque (na)

  // Commands 1 (homing) and 2 (init walking): quintic joint trajectory from the current reference
  // to a target over kTransitionDuration, tracked by the TSID posture task, and a quintic CoM
  // reference to the CoM of the target pose (both feet in contact). Between commands the last
  // target is held.
  static constexpr double kStandBaseHeight = 0.84;    // [m] base above the ground, command 2
  static constexpr double kTransitionDuration = 2.0;  // [s]
  Eigen::VectorXd qj_home_;               // command 1 target: all joints 0 (na)
  Eigen::VectorXd qj_stand_;              // command 2 target: IK, base kStandBaseHeight high (na)
  Eigen::Vector3d com_home_in_foot_;      // CoM of each target pose in the left foot frame
  Eigen::Vector3d com_stand_in_foot_;
  Eigen::VectorXd qj_start_, qj_target_;  // transition running / last one (na)
  Eigen::VectorXd qj_ref_, dqj_ref_, ddqj_ref_;     // trajectory sample of this cycle (na)
  Eigen::VectorXd com_start_, com_target_;          // world (3)
  Eigen::VectorXd com_ref_, dcom_ref_, ddcom_ref_;  // sample of this cycle (3)
  // [s] update time of the last start; -inf = none yet (the targets are held)
  double transition_start_time_ = -std::numeric_limits<double>::infinity();
  pinocchio::Data data_;                  // FK of the left foot in startTransition
  pinocchio::FrameIndex left_foot_id_ = 0;

  // Walking (command 3)
  enum class Mode { STAND, WALK };
  Mode mode_ = Mode::STAND;               // STAND: commands 1 / 2 (transitions); WALK: walk planner
  bool in_walking_pose_ = false;          // last transition was init walking (walking allowed)
  static constexpr double kRestTolerance = 0.002; // [m] planned CoM - CP at rest -> standing again
  static constexpr double kMinTouchdownPhase = 0.7; // use_time_base false: earlier touchdowns ignored
  bool stop_requested_ = false;           // the planner was asked to stop (this walk)
  Eigen::VectorXd q_des_, v_des_;         // joint command while walking: the measured joint state
                                          // integrated one cycle with the TSID accelerations (na)
  Eigen::VectorXd drive_kp_, drive_kd_;   // drive gains per joint (na), loadDriveGains
  pinocchio::SE3 swing_pose_;             // swing foot reference of this cycle (world)
  pinocchio::SE3 base_pose_ = pinocchio::SE3::Identity(); // torso orientation reference while walking (yaw)
  Eigen::Matrix<double, 6, 1> base_vel_ = Eigen::Matrix<double, 6, 1>::Zero(), base_acc_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> swing_vel_, swing_acc_; // [linear; angular]
  Eigen::Matrix<double, 6, 1> zero6_ = Eigen::Matrix<double, 6, 1>::Zero();

  // Commands of the caller, read by the next update()
  bool command_pending_ = false;
  MotionCommand command_ = MotionCommand::HOMING;
  int walk_steps_ = 0;                    // full steps of this walk, 0 = until stopWalking()
  bool stop_command_ = false;             // stopWalking()
  double turn_cmd_ = 0.0;                 // last turn command in [-1, 1]
};

}  // namespace biped_controllers
