#include "estimation/contact_fusion.h"
#include "common/data_type.h"

#include <algorithm>                        // std::clamp
#include <cmath>                            // std::exp, M_PI
#include <pinocchio/algorithm/crba.hpp>     // pinocchio::crba (mass matrix)
#include <pinocchio/algorithm/rnea.hpp>     // computeCoriolisMatrix, computeGeneralizedGravity
#include <pinocchio/algorithm/jacobian.hpp> // computeJointJacobians
#include <pinocchio/algorithm/frames.hpp>   // updateFramePlacements, getFrameJacobian
#include <pinocchio/algorithm/center-of-mass.hpp> // computeTotalMass
#include <Eigen/SVD>
#include <stdexcept>

// Constructor
ContactFusion::ContactFusion(const pinocchio::Model &model, const std::vector<std::string> &foot_frames,
                             double dt, double lamda)
    : contact_filter(makeContactFilter(static_cast<int>(foot_frames.size()))), // no default constructor
      model_(model), data_(model) // reference and Data have no default constructor
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
    // Physical range of the vertical foot force: the ground only pushes, and a
    // walking step stays far below 3 m g
    this->f_z_max_ = 3.0 * pinocchio::computeTotalMass(model) * 9.81;

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
        this->sole_corners_.push_back(Eigen::Matrix3Xd::Zero(3, 1)); // foot frame origin until setSoleCorners()
        this->foot_height_.push_back(0.0);
    }

    // The robot starts standing on all feet: initial contact probability 1
    const int n_feet = static_cast<int>(foot_frames.size());
    this->contact_filter.init(VectorXd::Ones(n_feet));
    this->u_ = VectorXd::Ones(n_feet);
    this->z_ = VectorXd::Ones(2 * n_feet);
    this->p_fused_ = VectorXd::Ones(n_feet);
    this->contact_state_.assign(n_feet, true);
}

// Contact probability Kalman filter (paper eq. 12-25). With A = 0 the
// prediction is the phase-based prior alone (x = u, P = Sigma_w), so the
// update is a per-foot inverse-variance weighted average of the prior and the
// two measurements.
KalmanFilter ContactFusion::makeContactFilter (int n_feet)
{
    const MatrixXd I = MatrixXd::Identity(n_feet, n_feet);
    const MatrixXd A = MatrixXd::Zero(n_feet, n_feet); // eq. 16: no state carried over
    const MatrixXd B = I;                              // eq. 16: input = phase-based prior
    MatrixXd C(2 * n_feet, n_feet);                    // eq. 25: H = [I; I]
    C << I, I;
    const MatrixXd P0 = I;                             // initial estimate covariance
    MatrixXd R = MatrixXd::Zero(2 * n_feet, 2 * n_feet); // eq. 24: blkdiag(Sigma_v1, Sigma_v2)
    R.topLeftCorner(n_feet, n_feet) = kRHeight_ * I;
    R.bottomRightCorner(n_feet, n_feet) = kRForce_ * I;
    const MatrixXd Q = kQPhase_ * I;                   // eq. 15: Sigma_w
    return KalmanFilter(A, B, C, P0, R, Q);
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

double ContactFusion::calculate_phase_contact_probability(double phi, bool scheduled_contact)
{
    const int s_phi = int(scheduled_contact); // 1/True = contact, 0/False = swing
    const int s_phi_bar = int(1-s_phi);
    // expected phase value at which the contact state switches
    const double muy_c0 = 0; // expected to be in swing as phi = 0
    const double muy_c1 = 1; // expected to be in contact as phi = 1

    double p_stand = (std::erf((phi - muy_c0)/(this->sigma_phase_c_*std::sqrt(2)))
     + std::erf((muy_c1 - phi)/(this->sigma_phase_c_*std::sqrt(2))));

    double p_swing = 2.0 + std::erf((muy_c0 - phi)/(this->sigma_phase_c_*std::sqrt(2)))
    + std::erf((phi-muy_c1)/(this->sigma_phase_c_*std::sqrt(2)));
    return 0.5*(s_phi * p_stand + s_phi_bar * p_swing);
}

double ContactFusion::calculate_force_contact_probability(double fz)
{
    double erf = std::erf((fz - this->muy_fc_)/(this->sigma_fc_*std::sqrt(2)));
    return 0.5*(1+erf);
}

// Eq. 20: P(c | pz) = 1/2 [1 + erf((muy_zg - pz) / (sigma_zg sqrt(2)))],
// high when the sole is at or below the expected ground height
double ContactFusion::calculate_height_contact_probability(double pz)
{
    double erf = std::erf((this->muy_zg_ - pz)/(this->sigma_zg_*std::sqrt(2)));
    return 0.5*(1+erf);
}

// Contact probability fusion (paper Sec. IV, eq. 12-19, 26)
const VectorXd &ContactFusion::fuseContactProbability (const std::vector<bool> &s_phi,
                                                      const std::vector<double> &phi)
{
    const int n = static_cast<int>(foot_ids_.size());
    for (int i = 0; i < n; ++i)
    {
        u_(i) = calculate_phase_contact_probability(phi[i], s_phi[i]);         // eq. 14
        z_(i) = calculate_height_contact_probability(foot_height_[i]);         // eq. 21
        z_(n + i) = calculate_force_contact_probability(foot_wrench_[i](2));   // eq. 23
    }

    contact_filter.predict(u_); // eq. 12-13: x = u, P = Sigma_w (A = 0)
    contact_filter.update(z_);  // eq. 17-19
    // The weighted average of probabilities stays in [0, 1]; clamp rounding errors
    p_fused_ = contact_filter.get_state().cwiseMax(0.0).cwiseMin(1.0);

    // Contact decision with hysteresis (eq. 26), so P hovering around the
    // threshold does not toggle the contact state every tick
    for (int i = 0; i < n; ++i)
    {
        if (!contact_state_[i] && p_fused_(i) > p_on_) contact_state_[i] = true;
        else if (contact_state_[i] && p_fused_(i) < p_off_) contact_state_[i] = false;
    }
    return p_fused_;
}

// Per-foot contact wrench (paper Sec. III-C): f_i = (S_i J_i^T)^-1 S_i tau_d,
// assuming every disturbance on a leg's joints comes from its foot
void ContactFusion::computeFootWrenches (const VectorXd &q)
{
    pinocchio::computeJointJacobians(model_, data_, q); // also runs the forward kinematics
    pinocchio::updateFramePlacements(model_, data_);

    for (size_t i = 0; i < foot_ids_.size(); ++i)
    {
        // Sole height: lowest corner in the world frame, so a heel or toe strike
        // on a tilted foot counts as soon as one corner reaches the ground
        const pinocchio::SE3 &oMf = data_.oMf[foot_ids_[i]];
        foot_height_[i] = ((oMf.rotation() * sole_corners_[i]).colwise() + oMf.translation()).row(2).minCoeff();

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
        // Damped least squares: w = V diag(s / (s^2 + lambda^2)) U^T (S_i tau_d).
        // Near a straight-knee singularity S_i J_i^T has a tiny singular value;
        // an exact solve (inverse / QR) divides the observer noise by it and the
        // estimate blows up (|f| ~ 1e5 N seen at the end of stance). Damping
        // drops those directions instead.
        const Eigen::JacobiSVD<MatrixXd> svd(SJt, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const VectorXd s = svd.singularValues();
        const VectorXd s_damped = s.cwiseQuotient((s.array().square() + wrench_damping_ * wrench_damping_).matrix());
        foot_wrench_[i] = svd.matrixV() * s_damped.asDiagonal() * svd.matrixU().transpose() * Stau_d;
        foot_wrench_[i](2) = std::clamp(foot_wrench_[i](2), 0.0, f_z_max_);
    }
}
