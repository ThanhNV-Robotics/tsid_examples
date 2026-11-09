#include "control/controller_params.h"

#include <cstdio>
#include <stdexcept>
#include <type_traits>

#include <yaml-cpp/yaml.h>

namespace biped_controllers
{

void ControllerParams::load(const std::string &controllers_yaml_path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(controllers_yaml_path);
  }
  catch (const std::exception &e)
  {
    throw std::runtime_error("ControllerParams: cannot load '" + controllers_yaml_path + "': " + e.what());
  }
  const YAML::Node cfg = root["wbc_controller"]["ros__parameters"];
  if (!cfg || !cfg.IsMap())
  {
    throw std::runtime_error("ControllerParams: no wbc_controller / ros__parameters in '" + controllers_yaml_path + "'");
  }

  // a key that is missing keeps its default; a value of the wrong type throws with the key name
  const auto read = [&](const char *key, auto &value) {
    if (!cfg[key]) return;
    try
    {
      value = cfg[key].as<std::decay_t<decltype(value)>>();
    }
    catch (const YAML::Exception &e)
    {
      throw std::runtime_error(std::string("ControllerParams: '") + key + "' in '" + controllers_yaml_path +
                               "' has the wrong type: " + e.what());
    }
  };
  read("left_foot_frame", this->left_foot_frame);
  read("right_foot_frame", this->right_foot_frame);
  read("use_ground_truth", this->use_ground_truth);
  read("torque_mode", this->torque_mode);
  read("walk_vx", this->walk_vx);
  read("max_yaw_rate", this->max_yaw_rate);
  read("use_time_base", this->use_time_base);
  read("touchdown_confirm_time", this->touchdown_confirm_time);
  read("late_touchdown_timeout", this->late_touchdown_timeout);

  // forward walking only (the planner clamps the step to [0, max_step_length])
  if (this->walk_vx < 0.0)
  {
    throw std::runtime_error("ControllerParams: walk_vx must be >= 0 (forward walking only)");
  }
  if (this->max_yaw_rate < 0.0 || this->touchdown_confirm_time < 0.0 || this->late_touchdown_timeout < 0.0)
  {
    throw std::runtime_error("ControllerParams: max_yaw_rate, touchdown_confirm_time and "
                             "late_touchdown_timeout must be >= 0");
  }
}

void ControllerParams::print() const
{
  std::printf("[WBC] URDF: %s\n[WBC] config: %s\n[WBC] gains: %s\n", this->urdf_path.c_str(), this->config_path.c_str(),
              this->gains_path.c_str());
  std::printf("[WBC] walk_vx: %.3f m/s%s, max_yaw_rate: %.3f rad/s\n", this->walk_vx,
              this->walk_vx == 0.0 ? " (stepping in place)" : "", this->max_yaw_rate);
  std::printf("[WBC] use_time_base: %s\n", this->use_time_base
                                               ? "true (step ends after Tswing)"
                                               : "false (event based: steps end at the touchdown of the contact estimator)");
  if (!this->use_time_base)
  {
    std::printf("[WBC] touchdown_confirm_time: %.3f s, late_touchdown_timeout: %.3f s%s\n", this->touchdown_confirm_time,
                this->late_touchdown_timeout, this->late_touchdown_timeout == 0.0 ? " (none)" : "");
  }
  std::printf("[WBC] torque_mode: %s\n", this->torque_mode ? "true (drive kp = kd = 0, tau = TSID torque)"
                                                           : "false (PVT, tau_ff = TSID torque - drive PD)");
  std::printf("[WBC] base state: %s\n", this->use_ground_truth ? "simulator ground truth" : "state estimator");
}

}  // namespace biped_controllers
