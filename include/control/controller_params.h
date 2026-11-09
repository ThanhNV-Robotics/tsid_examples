#pragma once

#include <string>

namespace biped_controllers
{

// Options of WBCController, the wbc_controller parameters of the ROS 2 project. The member values
// are the defaults; a value in controllers.yaml overrides them.
struct ControllerParams
{
  // files: the URDF (robot model, sole geometry), the WBC yaml (TSID tasks, contacts, LIP planner)
  // and the drive gains
  std::string urdf_path;
  std::string config_path;
  std::string gains_path;
  // URDF frames of the feet (state estimator, contact fusion, TSID contacts)
  std::string left_foot_frame = "left_ankle_pitch_link";
  std::string right_foot_frame = "right_ankle_pitch_link";
  // "cheat" estimator: base pose and twist from the simulator's ground truth (WBCController::update)
  // instead of the state estimator
  bool use_ground_truth = false;
  // pure torque control: the drive applies the TSID torque (kp = kd = 0). false: PVT command
  // (position, velocity, feedforward torque) with the drive gains
  bool torque_mode = false;
  // [m/s] forward walking speed (step length = walk_vx * Tswing, clamped to [0, max_step_length] of
  // the planner yaml); 0 = stepping in place
  double walk_vx = 0.0;
  // [rad/s] yaw rate at full turn command (WBCController::setTurnCommand, left turn positive)
  double max_yaw_rate = 0.2;
  // contact switching while walking. true: time based, the swing foot lands at the end of the step
  // (Tswing). false: event based (contact FSM of Bledt et al. ICRA 2018, Sec. VI): a contact of the
  // contact estimator holds the foot where it is (early touchdown) and lands it in TSID once it has
  // held for touchdown_confirm_time; without contact at Tswing the step waits (late touchdown, the
  // foot keeps descending) until the contact
  bool use_time_base = true;
  // [s] use_time_base false: an early contact must hold this long before the foot carries load
  double touchdown_confirm_time = 0.025;
  // [s] use_time_base false: longest wait for a late touchdown; then the foot lands in TSID anyway
  // (in the air). 0 = no timeout, wait for the contact
  double late_touchdown_timeout = 0.0;

  // reads section wbc_controller / ros__parameters of the yaml (a missing key keeps its default).
  // Throws std::runtime_error if the file or a value is invalid.
  void load(const std::string &controllers_yaml_path);

  // prints the values in effect, to see whether the yaml or the defaults were used
  void print() const;
};

}  // namespace biped_controllers
