#pragma once

#include <mujoco/mujoco.h>

#include "data_type.h"

// "Cheating" state estimator: instead of fusing IMU / kinematics / contacts,
// it reads the ground-truth floating-base state directly from MuJoCo.
// Joint states come from the RobotSensor reading; the base pose and twist
// (which no real sensor measures directly) come from the model's free joint.
class Cheat_StateEstimator {
public:
    Cheat_StateEstimator(const mjModel* model, const mjData* data);

    RobotState estimate(const RobotSensor& sensor) const;

private:
    const double contact_force_level = 20; // N
    const mjModel* mj_model{nullptr};
    const mjData* mj_data{nullptr};
    int base_qpos_adr{-1}; // qpos address of the free joint, -1 if fixed base
    int base_qvel_adr{-1}; // qvel address of the free joint
};
