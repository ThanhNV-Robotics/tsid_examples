#pragma once

#include "common/data_type.h"

#include <string>
#include <vector>
#include <pinocchio/multibody/model.hpp>    // pinocchio::Model
#include <pinocchio/multibody/data.hpp>     // pinocchio::Data

// Discrete-time generalized-momentum disturbance observer (Bledt et al.,
// "Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains",
// ICRA 2018, Sec. III) and the per-foot contact wrench estimated from it.
// See doc/contact_fusion_plan.md.
class ContactFusion
{
    public:
    // constructor
        // model: robot model with free-flyer (robot.model()), must outlive this object
        // foot_frames: contact frames, one per foot (e.g. {left ankle, right ankle})
        // dt: sampling time [s], lamda: observer cutoff frequency [Hz]
        ContactFusion (const pinocchio::Model &model, const std::vector<std::string> &foot_frames,
                       double dt, double lamda);
        // q (nq), v (nv): Pinocchio state of this tick, from robotStateToPinocchio()
        // tau: applied joint torques (na), e.g. RobotSensor::actuator_state.torquej
        // Also updates the foot wrenches, see getFootWrench()
        const VectorXd &computeDisturbanceTorque (const VectorXd &q, const VectorXd &v,
                                                  const VectorXd &tau);

        // Latest disturbance torque estimate (nv)
        const VectorXd &getDisturbanceTorque () const { return tau_d_; }
        // Latest external wrench [f; m] on foot i (order of foot_frames), applied by
        // the ground on the robot, world-aligned axes at the foot frame origin.
        // f_z = getFootWrench(i)(2) is the estimated normal force.
        const Vector6d &getFootWrench (size_t i) const { return foot_wrench_[i]; }

    private:
        // Per-foot wrench: (S_i J_i^T)^-1 S_i tau_d, S_i = velocity indices of the leg joints
        void computeFootWrenches (const VectorXd &q);

        const pinocchio::Model &model_;
        pinocchio::Data data_; // own Data: tsid.data() still holds the previous tick here
        double dt_ = 0.001; // sampling time
        double lamda_ = 15; // Hz, cutoff frequency of the torque disturbance observer (converted to rad/s)
        double gamma_ = 0.0; // z-domain pole of the low-pass filter, set in the constructor
        double beta_ = 0.0;  // (1 - gamma) / (gamma dt), set in the constructor
        VectorXd y_; // Low-pass filter state
        VectorXd tau_d_; // estimated disturbance torque

        std::vector<pinocchio::FrameIndex> foot_ids_;
        std::vector<std::vector<int>> leg_dofs_; // velocity indices of the joints between base and foot
        std::vector<Vector6d> foot_wrench_;
        Jacobian6 J_; // foot frame Jacobian buffer (6 x nv)

        bool initialized_ = false;
};
