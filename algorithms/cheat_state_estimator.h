#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "data_type.h"

// "Cheating" state estimator: instead of fusing IMU / kinematics / contacts,
// it reads the ground-truth floating-base state directly from MuJoCo.
// Joint states come from the RobotSensor reading; the base pose and twist
// (which no real sensor measures directly) come from the model's free joint.
// Foot contact also comes from the simulator: the contact forces MuJoCo
// computed between each foot and the environment (mj_contactForce). No foot
// sensor is read, so this is the ground truth for a sensorless contact estimator.
class Cheat_StateEstimator {
public:
    // foot_bodies: MuJoCo bodies of the feet, order [left, right]
    Cheat_StateEstimator(const mjModel* model, const mjData* data,
                         const std::vector<std::string>& foot_bodies = {"left_ankle_pitch_link",
                                                                        "right_ankle_pitch_link"});

    // Not const: stores the result as the latest state
    RobotState estimate(const RobotSensor& sensor);

    // Latest state computed by estimate()
    const RobotState& getState() const { return robot_state_; }

    // Ground-truth contact force on each foot (order of foot_bodies), applied by
    // the environment on the foot, world frame, summed over all contact points.
    // Updated by estimate(). [i](2) is the vertical force.
    const std::vector<Vector3d>& getContactForces() const { return contact_force_W_; }

private:
    // Sum the forces of all contacts between each foot and anything outside the
    // robot (floor, terrain); self-collisions are ignored
    void computeContactForces();

    RobotState robot_state_;
    const double contact_force_level = 20; // N
    const mjModel* mj_model{nullptr};
    const mjData* mj_data{nullptr};
    int base_qpos_adr{-1}; // qpos address of the free joint, -1 if fixed base
    int base_qvel_adr{-1}; // qvel address of the free joint
    std::vector<int> foot_body_ids_;          // MuJoCo body id of each foot
    std::vector<Vector3d> contact_force_W_;   // ground-truth force on each foot, world frame
};
