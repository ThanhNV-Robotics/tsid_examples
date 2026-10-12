#include "estimation/contact_fusion.h"
#include "common/data_type.h"

#include <cmath>                            // std::exp, M_PI
#include <pinocchio/algorithm/crba.hpp>     // pinocchio::crba (mass matrix)
#include <pinocchio/algorithm/rnea.hpp>     // computeCoriolisMatrix, computeGeneralizedGravity
#include <pinocchio/algorithm/jacobian.hpp> // computeJointJacobians
#include <pinocchio/algorithm/frames.hpp>   // updateFramePlacements, getFrameJacobian
#include <stdexcept>

// Constructor
ContactFusion::ContactFusion(const pinocchio::Model &model, const std::vector<std::string> &foot_frames,
                             double dt, double lamda)
    : model_(model), data_(model) // reference and Data have no default constructor
{
    // Input: dt -> sampling time [s]
    //        lamda -> cutoff frequency [Hz]
    this->dt_ = dt;
    this->lamda_ = lamda;

    // Discrete-time observer constants (paper eq. 10): z-domain pole of the
    // low-pass filter, gamma = exp(-lambda dt) with lambda in rad/s
    this->gamma_ = std::exp(-2.0 * M_PI * this->lamda_ * this->dt_);
    this->beta_ = (1.0 - this->gamma_) / (this->gamma_ * this->dt_);

    this->y_ = VectorXd::Zero(model.nv);
    this->tau_d_ = VectorXd::Zero(model.nv);
    this->J_ = Jacobian6::Zero(6, model.nv);

    // Leg joints of each foot: every actuated joint on the path from the base
    // to the foot frame (model.supports = joints from the root to that joint).
    // The free-flyer (velocity indices 0..5) is not part of the leg.
    for (const std::string &name : foot_frames)
    {
        if (!model.existFrame(name))
            throw std::runtime_error("ContactFusion: unknown foot frame " + name);
        const pinocchio::FrameIndex fid = model.getFrameId(name);
        std::vector<int> dofs;
        for (const pinocchio::JointIndex j : model.supports[model.frames[fid].parentJoint])
        {
            const int idx_v = model.joints[j].idx_v();
            if (j == 0 || idx_v < 6) continue; // universe and free-flyer
            for (int k = 0; k < model.joints[j].nv(); ++k) dofs.push_back(idx_v + k);
        }
        this->foot_ids_.push_back(fid);
        this->leg_dofs_.push_back(dofs);
        this->foot_wrench_.push_back(Vector6d::Zero());
    }
}

// Disturbance torque estimation
const VectorXd &ContactFusion::computeDisturbanceTorque (const VectorXd &q, const VectorXd &v,
                                                  const VectorXd &tau)
{
    pinocchio::crba(model_, data_, q); // compute dynamics term 
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeCoriolisMatrix(model_, data_, q, v); // Compute Corriolis matrix
    pinocchio::computeGeneralizedGravity(model_, data_, q); // g(q) compute gravity vector

    // Compute generalized Momentumn
    const VectorXd p = data_.M * v;
    // Selection matrix for joint torque vector
    VectorXd St = VectorXd::Zero(model_.nv);
    St.tail(tau.size()) = tau; // S^T: no torque on the 6 base DoF
    const VectorXd x = beta_ * p + St + data_.C.transpose() * v - data_.g;

    if (!initialized_) // first time step
    {y_ = x; initialized_ = true;}

    y_ = gamma_ * y_ + (1.0 - gamma_) * x;
    tau_d_ = beta_ * p - y_;

    computeFootWrenches(q);
    return tau_d_;
}

// Per-foot contact wrench (paper Sec. III-C): f_i = (S_i J_i^T)^-1 S_i tau_d,
// assuming every disturbance on a leg's joints comes from its foot
void ContactFusion::computeFootWrenches (const VectorXd &q)
{
    pinocchio::computeJointJacobians(model_, data_, q); // also runs the forward kinematics
    pinocchio::updateFramePlacements(model_, data_);

    for (size_t i = 0; i < foot_ids_.size(); ++i)
    {
        J_.setZero();
        // LOCAL_WORLD_ALIGNED: wrench at the foot frame origin, world axes (f_z = vertical)
        pinocchio::getFrameJacobian(model_, data_, foot_ids_[i], pinocchio::LOCAL_WORLD_ALIGNED, J_);

        const std::vector<int> &dofs = leg_dofs_[i];
        const int n = static_cast<int>(dofs.size());
        MatrixXd SJt(n, 6); // S_i J_i^T: 6 x 6 for a 6-DoF leg
        VectorXd Stau_d(n); // S_i tau_d
        for (int k = 0; k < n; ++k)
        {
            SJt.row(k) = J_.col(dofs[k]).transpose();
            Stau_d(k) = tau_d_(dofs[k]);
        }
        // QR instead of a plain inverse: stays usable near a straight-knee singularity
        foot_wrench_[i] = SJt.colPivHouseholderQr().solve(Stau_d);
    }
}
