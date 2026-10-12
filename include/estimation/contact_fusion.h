#pragma once

#include "common/data_type.h"

#include <string>
#include <vector>
#include <pinocchio/multibody/model.hpp>    // pinocchio::Model
#include <pinocchio/multibody/data.hpp>     // pinocchio::Data
#include "estimation/kalman_filter.h"

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

        double calculate_phase_contact_probability(double phi, bool scheduled_contact);
        double calculate_force_contact_probability(double fz);
        // Height-based contact probability (paper eq. 20) for sole height pz [m]
        double calculate_height_contact_probability(double pz);

        // Sole corners of foot i in its foot frame (3 x k), e.g. the TSID contact
        // points (tsidTaskParser::ContactInfo::contact_points). Without them the
        // foot frame origin is used as the only sole point.
        void setSoleCorners (size_t i, const Eigen::Matrix3Xd &corners) { sole_corners_[i] = corners; }
        // Latest sole height of foot i: world z of its lowest sole corner [m].
        // Updated by computeDisturbanceTorque(). Uses the base height of q, so on
        // a real robot it inherits the base-height estimate's drift.
        double getFootHeight (size_t i) const { return foot_height_[i]; }

        // One Kalman filter step of the contact fusion (paper Sec. IV), after
        // computeDisturbanceTorque() of the same tick:
        //   predict: u = phase-based prior P(c | s_phi, phi) of each foot
        //   update:  z = [P(c | p_z) of each foot; P(c | f_z) of each foot]
        // then the contact state with hysteresis (eq. 26).
        // s_phi[i]: scheduled contact of foot i (true = stance), phi[i]: its subphase
        const VectorXd &fuseContactProbability (const std::vector<bool> &s_phi,
                                                const std::vector<double> &phi);
        // Latest fused contact probability of foot i, in [0, 1]
        double getContactProbability (size_t i) const { return p_fused_(i); }
        // Latest estimated contact state of foot i (s_hat)
        bool getContactState (size_t i) const { return contact_state_[i]; }
        // Inputs of the latest fusion step, for logging / tuning
        double getPhaseProbability (size_t i) const { return u_(i); }
        double getHeightProbability (size_t i) const { return z_(i); }
        double getForceProbability (size_t i) const { return z_(u_.size() + i); }

    private:
        // Contact probability fusion (paper Sec. IV): state x = [P_1(c) ... P_N(c)],
        // A = 0, B = I (prediction = phase-based prior), C = H = [I; I]
        // (measurements: foot height, foot force). Built by makeContactFilter().
        KalmanFilter contact_filter;
        const double sigma_phase_c_ = 0.1; // standard deviation sigma of the phase-based contact probability (variance sigma^2 = 0.01; paper Table I: sigma^2 = 0.05)
        const double muy_fc_ = 20; // N: expected (threadsold) contact force for contact dectection
        const double sigma_fc_ = 10; // N: standard deviation of the force-base contact probabbility
        // Height model: muy_zg_ = sole height at which contact is 50/50. Slightly
        // above the floor (z = 0), so a flat foot on the floor gives P ~ 0.98
        // (with muy_zg_ = 0 it would give exactly 0.5). Later: adapt it to the
        // sole height of the last touchdown for uneven terrain (paper Sec. IV-B.1).
        const double muy_zg_ = 0.01;    // m
        const double sigma_zg_ = 0.005; // m: standard deviation, ~terrain roughness
        // Kalman filter covariances, tuned values of the paper (Sec. V-A)
        static constexpr double kQPhase_ = 0.998;  // Sigma_w: trust in the phase-based prior
        static constexpr double kRHeight_ = 0.841; // Sigma_v1: foot-height measurement
        static constexpr double kRForce_ = 0.930;  // Sigma_v2: foot-force measurement
        // Contact filter for n_feet feet with the matrices above
        static KalmanFilter makeContactFilter (int n_feet);
        // Hysteresis of the contact decision (eq. 26): contact when P rises above
        // p_on_, no contact when it falls below p_off_
        const double p_on_ = 0.5;
        const double p_off_ = 0.4;
        VectorXd u_;      // phase-based prior (N)
        VectorXd z_;      // measurements [height (N); force (N)]
        VectorXd p_fused_; // fused contact probability (N)
        std::vector<bool> contact_state_; // s_hat per foot

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
        std::vector<Eigen::Matrix3Xd> sole_corners_; // per foot, in the foot frame
        std::vector<double> foot_height_;            // per foot, world z of the lowest sole corner
        Jacobian6 J_; // foot frame Jacobian buffer (6 x nv)

        bool initialized_ = false;

};
