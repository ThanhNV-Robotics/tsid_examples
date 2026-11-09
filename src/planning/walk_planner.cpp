#include "planning/walk_planner.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace biped_controllers {
    // yaw of a rotation (z up), and an angle wrapped to [-pi, pi]
    static double yawOf(const Eigen::Matrix3d &R) { return std::atan2(R(1, 0), R(0, 0)); }
    static double wrapAngle(double a) { return std::remainder(a, 2.0 * M_PI); }

    // Reads the step settings from the "LIPPlanner" section of the yaml (the LIP keys are read by
    // LIPTaskPlanner), e.g.
    //   max_step_length: 0.3   # [m]
    //   double_support_time: 0.2 # [s] double support at the start of each step (0: instant switch)
    //   step_height: 0.05      # [m] swing foot peak height
    //   swing_landing_depth: 0.01 # [m] end of the swing trajectory below the foothold
    //   sole_x_min: -0.04      # [m] ZMP limits in single support, stance ankle frame
    //   sole_x_max: 0.12
    //   sole_y_half: 0.02
    //   late_descent_speed: 0.2       # [m/s] late touchdown: swing foot descent below the foothold
    //   late_descent_ramp_time: 0.05  # [s] the descent speed ramps up from 0
    //   late_max_depth: 0.02          # [m] deepest target below the foothold
    //   foothold_adaptation: true     # CP-based foothold adaptation
    //   foothold_max_width_change: 0.05 # [m] lateral range around the nominal foothold
    //   foothold_max_rate: 0.5        # [m/s] largest foothold change per second
    //   foothold_freeze_phase: 0.8    # the foothold no longer changes after this step phase
    //   max_turn_per_step: 0.2        # [rad] largest heading change of a step
    // A key that is missing keeps its default from the header. The stance width is measured at
    // reset() (the hip_width key is not used).
    WalkPlanner::WalkPlanner(tsid::robots::RobotWrapper &robot, const std::string &CONFIG_YAML_PATH,
                             const std::string &left_foot_frame, const std::string &right_foot_frame)
        : robot_(robot), data_(robot.model()), lip_(CONFIG_YAML_PATH)
    {
        // ankle frames of the support frames
        const pinocchio::Model &model = this->robot_.model();
        const std::string frames[2] = {left_foot_frame, right_foot_frame};
        for (int i = 0; i < 2; ++i)
        {
            if (!model.existFrame(frames[i]))
            {
                throw std::runtime_error("WalkPlanner: no frame '" + frames[i] + "' in the robot model");
            }
            this->foot_id_[i] = model.getFrameId(frames[i]);
        }

        // step settings (the file was already checked by LIPTaskPlanner)
        const YAML::Node cfg = YAML::LoadFile(CONFIG_YAML_PATH)["LIPPlanner"];
        const auto read = [&](const char *key, double &value) {
            if (!cfg[key]) return;
            try
            {
                value = cfg[key].as<double>();
            }
            catch (const YAML::Exception &e)
            {
                throw std::runtime_error(std::string("WalkPlanner: '") + key + "' in '" +
                                         CONFIG_YAML_PATH + "' is not a number: " + e.what());
            }
        };
        read("max_step_length", this->max_step_length_);
        read("double_support_time", this->t_ds_);
        read("step_height", this->step_height_);
        read("swing_landing_depth", this->swing_landing_depth_);
        read("sole_x_min", this->sole_x_min_);
        read("sole_x_max", this->sole_x_max_);
        read("sole_y_half", this->sole_y_half_);
        read("late_descent_speed", this->late_descent_speed_);
        read("late_descent_ramp_time", this->late_descent_ramp_time_);
        read("late_max_depth", this->late_max_depth_);
        read("foothold_max_width_change", this->foothold_max_width_change_);
        read("foothold_max_rate", this->foothold_max_rate_);
        read("foothold_freeze_phase", this->foothold_freeze_phase_);
        read("max_turn_per_step", this->max_turn_per_step_);
        if (cfg["foothold_adaptation"])
        {
            try
            {
                this->foothold_adaptation_ = cfg["foothold_adaptation"].as<bool>();
            }
            catch (const YAML::Exception &e)
            {
                throw std::runtime_error("WalkPlanner: 'foothold_adaptation' in '" + CONFIG_YAML_PATH +
                                         "' is not true / false: " + e.what());
            }
        }
        if (this->max_step_length_ < 0.0 || this->step_height_ < 0.0 || this->sole_x_min_ > this->sole_x_max_ ||
            this->sole_y_half_ < 0.0 || this->late_descent_speed_ < 0.0 || this->foothold_max_width_change_ < 0.0 ||
            this->swing_landing_depth_ < 0.0 || this->t_ds_ < 0.0 || this->max_turn_per_step_ < 0.0 ||
            this->foothold_max_rate_ < 0.0)
        {
            throw std::runtime_error("WalkPlanner: max_step_length, step_height, sole_y_half, late_descent_speed, "
                                     "foothold_max_width_change, foothold_max_rate, swing_landing_depth, double_support_time and max_turn_per_step must be >= 0, "
                                     "sole_x_min <= sole_x_max (file '" + CONFIG_YAML_PATH + "')");
        }

        // one phase of the LIP = one swing
        this->T_ = this->lip_.getTswing();
    }

    // support frame on an ankle: its position, yaw of its orientation only (z up)
    pinocchio::SE3 WalkPlanner::supportFrame(const Eigen::VectorXd &q, bool left)
    {
        // ankle frames of q (both: the caller may also need the other foot)
        const pinocchio::Model &model = this->robot_.model();
        pinocchio::forwardKinematics(model, this->data_, q);
        pinocchio::updateFramePlacement(model, this->data_, this->foot_id_[0]);
        pinocchio::updateFramePlacement(model, this->data_, this->foot_id_[1]);
        const pinocchio::SE3 &oMa = this->data_.oMf[this->foot_id_[left ? 0 : 1]];

        // yaw only: x forward along the foot, z up
        const double yaw = std::atan2(oMa.rotation()(1, 0), oMa.rotation()(0, 0));
        return pinocchio::SE3(Eigen::AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix(), oMa.translation());
    }

    void WalkPlanner::reset(const Eigen::VectorXd &q)
    {
        // double support: support frame on the left ankle; ankle height above the ground (z = 0 in
        // the estimator), so that the CoM is zc above the ground
        this->left_stance_ = true;
        this->oMs_ = this->supportFrame(q, true); // set support frame to left ankle
        this->ankle_height_ = this->oMs_.translation().z();

        // right foot standing where it is (support frame), at rest
        this->swing_start_pos_ = this->oMs_.actInv(this->data_.oMf[this->foot_id_[1]].translation());
        this->foothold_ = this->swing_start_pos_;
        this->foot_nominal_ = this->swing_start_pos_;
        this->swing_pos_l_ = this->swing_start_pos_;
        this->swing_vel_l_.setZero();
        this->swing_acc_l_.setZero();

        // yaw: right foot relative to the left one; heading and torso between the feet
        const double frame_yaw = yawOf(this->oMs_.rotation());
        this->swing_yaw_start_l_ = wrapAngle(yawOf(this->data_.oMf[this->foot_id_[1]].rotation()) - frame_yaw);
        this->swing_yaw_l_ = this->swing_yaw_start_l_;
        this->swing_wz_l_ = 0.0;
        this->swing_dwz_l_ = 0.0;
        this->heading_ = frame_yaw + 0.5 * this->swing_yaw_start_l_;
        this->heading_err_ = 0.0;
        this->step_h_.setZero();

        // CoM at rest in the support frame, LIP at the CoM height above the ground, init phase next
        const Vector3d com = this->oMs_.actInv(pinocchio::centerOfMass(this->robot_.model(), this->data_, q));
        this->lip_.setComHeight(com.z() + this->ankle_height_);
        this->com_bias_ = com.head<2>() - 0.5 * this->swing_start_pos_.head<2>(); // middle: half the right ankle

        // stance width: distance of the ankles now (the footholds and CP targets lie on the feet)
        this->hip_width_ = this->swing_start_pos_.head<2>().norm();
        this->lip_.reset(com.head<2>());
        this->cp_meas_ = com.head<2>();
        this->phase_time_ = 0.0;
        this->in_ds_ = false;
        this->phase_ = WalkPhase::INIT;
        this->stop_requested_ = false;
        this->last_step_ = false;
        this->touchdown_ = false;
        this->step_count_ = 0;
        this->late_ = false;
        this->late_time_ = 0.0;

        this->toWorld(); // transform swing trajectory to global frame
    }

    void WalkPlanner::setVelocityCommand(double vx)
    {
        this->step_length_ = std::clamp(vx * this->T_, 0.0, this->max_step_length_);
    }

    // main function run in control update()
    void WalkPlanner::planWalking(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        // CP feedback: CoM and CP of the robot in the support frame
        this->measureCP(q, v);

        // CoM: one LIP cycle toward the CP target of the current (sub)phase in its remaining time
        Vector2d cp_target, zmp_min, zmp_max;
        const double t_rem = this->phaseTarget(cp_target, zmp_min, zmp_max);
        this->lip_.planCycle(cp_target, t_rem, zmp_min, zmp_max); // use LIP planner to compute CoM reference motion
        this->phase_time_ += this->lip_.getDt(); // increase phase time a time step

        // events of this cycle
        if (this->inDoubleSupport())
        {
            // end of the double support: the trailing foot lifts off, its swing starts
            if (this->phase_time_ >= this->t_ds_)
            {
                this->in_ds_ = false;
                this->phase_time_ = 0.0;
                this->touchdown_ = false;
            }
        }
        else if (this->phase_ == WalkPhase::STEP)
        {
            // swing: ends at the touchdown (event based) or at Tswing (time based); event based
            // without touchdown at Tswing: late, the step waits
            const bool swing_end = this->phase_time_ >= this->T_;
            if (this->touchdown_ || (swing_end && !this->wait_touchdown_))
            {
                this->endStep(q, v);
            }
            else
            {
                this->adaptFoothold(cp_target); // the step goes on: foothold for the predicted CP
            }
        }
        else if (this->phase_time_ >= this->T_)
        {
            this->endOfPhase(q, v); // init / stop / rest
        }

        // late: the swing is past Tswing without touchdown
        this->late_ = this->isSwinging() && this->phase_time_ >= this->T_;
        this->late_time_ = this->late_ ? this->phase_time_ - this->T_ : 0.0;

        // swing foot of this cycle (both feet stand in the other phases); late: below the foothold
        if (this->isSwinging())
        {
            this->computeSwingTrajectory(this->getPhase());
            if (this->late_)
            {
                this->computeLateDescent();
            }
        }
        else
        {
            // both feet down: the other foot keeps its yaw
            this->swing_yaw_l_ = this->swing_yaw_start_l_;
            this->swing_wz_l_ = 0.0;
            this->swing_dwz_l_ = 0.0;
        }

        this->toWorld();
    }

    // CP target of the current (sub)phase (support frame), ZMP limits, remaining time. As
    // walk_planner.cpp of tsid_examples:
    //   init: the stance ankle shifted by the CoM bias (the CoM keeps its x), Tswing
    //   double support: CP at the start of the single support of the periodic gait (Stage 3b),
    //         t_ds; with E_ss = e^{w Tswing}, E_ds = e^{w t_ds}, relative to the stance ankle
    //           cp_start = ( s (1 + E_ds) / (2 (E_ds E_ss - 1)),  y_f (1 + E_ds) / (2 (1 + E_ds E_ss)) )
    //         (s, y_f: nominal step of the coming swing in the heading axes, then rotated by the
    //         heading error)
    //   swing: CP at the touchdown, E_ss cp_start (t_ds = 0: s b / (b - 1), y_f b / (b + 1)), to
    //         Tswing (0 once late); ZMP on the stance sole
    //   last swing: middle between the stance ankle and the foothold shifted by the CoM bias (the CP
    //         is between the feet at touchdown)
    //   stop / rest: middle of the feet shifted by the CoM bias, Tswing
    // The targets use the nominal foothold: the adapted one follows the CP (adaptFoothold). With
    // both feet down (init, double support, stop, rest) the ZMP is in the box around both soles
    double WalkPlanner::phaseTarget(Vector2d &cp_target, Vector2d &zmp_min, Vector2d &zmp_max) const
    {
        // double support limits: the stance ankle (origin) and the other standing ankle
        const Vector2d other = this->swing_start_pos_.head<2>();
        zmp_min << std::min(0.0, other.x()) + this->sole_x_min_, std::min(0.0, other.y()) - this->sole_y_half_;
        zmp_max << std::max(0.0, other.x()) + this->sole_x_max_, std::max(0.0, other.y()) + this->sole_y_half_;

        // periodic gait CP at the start of the single support (Stage 3b)
        const double w = this->lip_.getOmega();
        const double e_ss = std::exp(w * this->T_), e_ds = std::exp(w * this->t_ds_);
        // (heading axes, rotated by the heading error into the support frame)
        const Vector2d cp_start = Eigen::Rotation2Dd(this->heading_err_) *
                                  Vector2d(this->step_h_.x() * (1.0 + e_ds) / (2.0 * (e_ds * e_ss - 1.0)),
                                           this->step_h_.y() * (1.0 + e_ds) / (2.0 * (1.0 + e_ds * e_ss)));

        switch (this->phase_)
        {
        case WalkPhase::INIT:
            cp_target = this->com_bias_;
            return this->T_ - this->phase_time_;
        case WalkPhase::STEP:
            if (this->in_ds_)
            {
                cp_target = cp_start;
                return this->t_ds_ - this->phase_time_;
            }
            cp_target = this->last_step_ ? Vector2d(this->com_bias_ + 0.5 * this->foot_nominal_.head<2>())
                                         : Vector2d(e_ss * cp_start);
            zmp_min << this->sole_x_min_, -this->sole_y_half_;
            zmp_max << this->sole_x_max_, this->sole_y_half_;
            return std::max(this->T_ - this->phase_time_, 0.0);
        case WalkPhase::STOP:
        case WalkPhase::REST:
            break;
        }
        cp_target = this->com_bias_ + this->stop_target_;
        return this->T_ - this->phase_time_;
    }

    // INIT -> first step (right foot first, no double support: both feet were down) or STOP;
    // STOP -> REST; REST stays (repeats)
    void WalkPlanner::endOfPhase(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        this->phase_time_ = 0.0;
        switch (this->phase_)
        {
        case WalkPhase::INIT:
            if (this->stop_requested_)
            {
                this->startStop(); // no step: the right foot is still where reset() measured it
            }
            else
            {
                this->switchSupport(q, v, true);
                this->phase_ = WalkPhase::STEP;
                this->in_ds_ = false;
                this->touchdown_ = false;
            }
            break;
        case WalkPhase::STOP:
            this->phase_ = WalkPhase::REST;
            break;
        case WalkPhase::STEP:
        case WalkPhase::REST:
            break;
        }
    }

    // STEP -> STEP (support switch, double support first; the last step once a stop is requested)
    // or, after the last step, STOP (the landed foot stays down)
    void WalkPlanner::endStep(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        this->phase_time_ = 0.0;
        this->touchdown_ = false;
        ++this->step_count_;
        this->switchSupport(q, v, false);
        if (this->last_step_)
        {
            this->startStop();
            return;
        }
        if (this->stop_requested_)
        {
            // closing step: the swing foot lands beside the stance foot (no step length), the feet
            // end side by side
            this->last_step_ = true;
            this->step_h_.x() = 0.0;
            this->setNominalFoothold();
        }
        this->in_ds_ = this->t_ds_ > 0.0; // both feet loaded until the trailing foot lifts off
    }

    void WalkPlanner::startStop()
    {
        // the other foot stands at its liftoff position (support frame), CP between the ankles
        this->phase_ = WalkPhase::STOP;
        this->stop_target_ = 0.5 * this->swing_start_pos_.head<2>();
        this->swing_pos_l_ = this->swing_start_pos_;
        this->swing_vel_l_.setZero();
        this->swing_acc_l_.setZero();
    }

    // first_step: end of the init phase, the left foot stays the stance foot and the right one swings;
    // otherwise the landed foot becomes the stance foot
    void WalkPlanner::switchSupport(const Eigen::VectorXd &q, const Eigen::VectorXd &v, bool first_step)
    {
        // new stance foot and its support frame from the estimated state
        if (!first_step)
        {
            this->left_stance_ = !this->left_stance_;
        }
        const pinocchio::SE3 oMs_old = this->oMs_;

        
        this->oMs_ = this->supportFrame(q, this->left_stance_); // switch support frame left<->right

        // LIP state into the new frame (both frames yaw only: the planar part is exact)
        // A.actInv(B) = A.inverse() * B
        const pinocchio::SE3 sMs_old = this->oMs_.actInv(oMs_old);
        this->lip_.transformState(sMs_old.rotation().topLeftCorner<2, 2>(), sMs_old.translation().head<2>());

        // swing foot: lifts off where it is now, with its yaw (feet of supportFrame(), before
        // measureCP() below runs its own kinematics)
        const int swing = this->left_stance_ ? 1 : 0;
        const double frame_yaw = yawOf(this->oMs_.rotation());
        this->swing_start_pos_ = this->oMs_.actInv(this->data_.oMf[this->foot_id_[swing]].translation());
        this->swing_yaw_start_l_ = wrapAngle(yawOf(this->data_.oMf[this->foot_id_[swing]].rotation()) - frame_yaw);
        this->swing_yaw_l_ = this->swing_yaw_start_l_;

        // heading of this step: advanced by the yaw rate command over one step period, at most
        // max_turn_per_step from the stance foot (the heading cannot run away from the feet)
        const double turn = std::clamp(this->yaw_rate_cmd_ * (this->T_ + this->t_ds_), -this->max_turn_per_step_,
                                       this->max_turn_per_step_);
        this->heading_err_ = std::clamp(wrapAngle(this->heading_ + turn - frame_yaw), -this->max_turn_per_step_,
                                        this->max_turn_per_step_);
        this->heading_ = frame_yaw + this->heading_err_;

        // the swing foot lands step_length ahead and hip_width aside of the stance ankle along the
        // heading, at its height; the step starts from the nominal foothold, adapted during the step
        this->step_h_ << this->step_length_, this->left_stance_ ? -this->hip_width_ : this->hip_width_;
        this->setNominalFoothold();

        // touchdown: planned CoM and CP pulled toward the measured ones (new frame)
        this->measureCP(q, v);
        this->lip_.blendTowardMeasured();
    }

    void WalkPlanner::setNominalFoothold()
    {
        this->foot_nominal_.head<2>() = Eigen::Rotation2Dd(this->heading_err_) * this->step_h_;
        this->foot_nominal_.z() = 0.0;
        this->foothold_ = this->foot_nominal_;
    }

    void WalkPlanner::measureCP(const Eigen::VectorXd &q, const Eigen::VectorXd &v)
    {
        // CoM and its velocity (world), into the support frame (yaw only: planar part exact)
        pinocchio::centerOfMass(this->robot_.model(), this->data_, q, v);
        const Vector3d com_l = this->oMs_.actInv(this->data_.com[0]);
        const Vector3d vcom_l = this->oMs_.rotation().transpose() * this->data_.vcom[0];

        // CP of the LIP: c + dc / w
        this->cp_meas_ = com_l.head<2>() + vcom_l.head<2>() / this->lip_.getOmega();
        this->lip_.setMeasuredState(com_l.head<2>(), this->cp_meas_);
    }

    // Stage 3 of tsid_examples walk_planner.cpp: the ZMP can only move on the sole, the foothold
    // takes over the rest of the correction. With the predicted CP at the end of the step
    //   cp_end = p + e^{w t_rem} (cp_fb - p)
    // the next step starts like the planned one if its foot is where the CP target is relative to
    // the nominal foothold:  foothold = cp_end - (cp_target - foot_nominal). Unclamped ZMP:
    // cp_end = cp_target, the foothold is the nominal one. e^{w t_rem} is large early in the step,
    // so the foothold is clamped (step length, lateral range), rate limited, and frozen late in
    // the swing (the swing foot must land where it is going)
    void WalkPlanner::adaptFoothold(const Vector2d &cp_target)
    {
        if (!this->foothold_adaptation_ || this->late_ || this->getPhase() > this->foothold_freeze_phase_)
        {
            return;
        }

        // foothold for the predicted CP, in the limits around the nominal one (heading axes: step
        // length, lateral range around the stance width)
        const Eigen::Rotation2Dd R_err(this->heading_err_);
        const Vector2d cp_end = this->lip_.getPredictedEndCP();
        Vector2d target_h = R_err.inverse() * (cp_end - (cp_target - this->foot_nominal_.head<2>()));
        target_h.x() = std::clamp(target_h.x(), -this->max_step_length_, this->max_step_length_);
        target_h.y() = std::clamp(target_h.y(), this->step_h_.y() - this->foothold_max_width_change_,
                                  this->step_h_.y() + this->foothold_max_width_change_);
        const Vector2d target = R_err * target_h;

        // rate limit: the swing target moves smoothly
        const Vector2d step = target - this->foothold_.head<2>();
        const double max_step = this->foothold_max_rate_ * this->lip_.getDt();
        const double n = step.norm();
        this->foothold_.head<2>() += (n > max_step) ? Vector2d(step * (max_step / n)) : step;
    }

    // swing foot at phase phi (support frame), as the earlier planner: cycloid from
    // liftoff to the foothold, zero velocity at both ends, plus a z bump of step_height at phi = 0.5
    //   c(phi) = (2 pi phi - sin(2 pi phi)) / (2 pi),  bump(phi) = h (1 - cos(2 pi phi)) / 2
    // The cycloid ends swing_landing_depth below the foothold: the foot crosses the ground (the
    // foothold height) before phi = 1, still moving down, and lands there with an impact
    void WalkPlanner::computeSwingTrajectory(double phi)
    {
        // cycloid and bump with their derivatives w.r.t. phi
        const double w = 2.0 * M_PI;
        const double c = (w * phi - std::sin(w * phi)) / w;
        const double dc = 1.0 - std::cos(w * phi);
        const double ddc = w * std::sin(w * phi);
        const double h = this->step_height_;
        const double bump = 0.5 * h * (1.0 - std::cos(w * phi));
        const double dbump = 0.5 * h * w * std::sin(w * phi);
        const double ddbump = 0.5 * h * w * w * std::cos(w * phi);

        // position, and the time derivatives (dphi/dt = 1 / T)
        Vector3d delta = this->foothold_ - this->swing_start_pos_;
        delta.z() -= this->swing_landing_depth_; // end below the ground: landing with a downward velocity
        const double T2 = this->T_ * this->T_;
        this->swing_pos_l_ = this->swing_start_pos_ + c * delta;
        this->swing_pos_l_.z() += bump;
        this->swing_vel_l_ = (dc / this->T_) * delta;
        this->swing_vel_l_.z() += dbump / this->T_;
        this->swing_acc_l_ = (ddc / T2) * delta;
        this->swing_acc_l_.z() += ddbump / T2;

        // yaw: from the liftoff yaw to the heading on the same cycloid
        const double dyaw = wrapAngle(this->heading_err_ - this->swing_yaw_start_l_);
        this->swing_yaw_l_ = this->swing_yaw_start_l_ + c * dyaw;
        this->swing_wz_l_ = (dc / this->T_) * dyaw;
        this->swing_dwz_l_ = (ddc / T2) * dyaw;
    }

    // late touchdown, after computeSwingTrajectory(1) (foothold, at rest): the foot keeps going down
    // (as the earlier planner), speed ramped from 0 to late_descent_speed over the ramp
    // time (constant acceleration), then constant, at most late_max_depth below the foothold
    void WalkPlanner::computeLateDescent()
    {
        // depth below the foothold, its rate and acceleration at the late time t
        const double t = this->late_time_;
        const double v = this->late_descent_speed_;
        const double Tr = this->late_descent_ramp_time_;
        double depth, vel, acc;
        if (Tr > 0.0 && t < Tr)
        {
            depth = 0.5 * v * t * t / Tr; // ramp: constant acceleration v / Tr
            vel = v * t / Tr;
            acc = v / Tr;
        }
        else
        {
            depth = (Tr > 0.0 ? 0.5 * v * Tr : 0.0) + v * (t - std::max(Tr, 0.0)); // constant speed
            vel = v;
            acc = 0.0;
        }

        // bottomed out: held at the max depth
        if (this->late_max_depth_ > 0.0 && depth >= this->late_max_depth_)
        {
            depth = this->late_max_depth_;
            vel = 0.0;
            acc = 0.0;
        }
        this->swing_pos_l_.z() -= depth;
        this->swing_vel_l_.z() -= vel;
        this->swing_acc_l_.z() -= acc;
    }

    // support frame -> world: positions with the full transform, velocities / accelerations with the
    // rotation only (the frame is fixed during a phase)
    void WalkPlanner::toWorld()
    {
        const Eigen::Matrix3d &R = this->oMs_.rotation();

        // CoM: LIP (x, y) at zc above the ground = zc - ankle_height above the stance ankle
        Vector3d com_l = this->lip_.getRefCoMPosition();
        com_l.z() -= this->ankle_height_;
        this->com_pos_ = this->oMs_.act(com_l);
        this->com_vel_.noalias() = R * this->lip_.getRefCoMVelocity();
        this->com_acc_.noalias() = R * this->lip_.getRefCoMAcceleration();

        // swing foot ankle: position, flat orientation at its yaw; torso yaw between the feet
        const double frame_yaw = yawOf(R);
        this->swing_rot_ = Eigen::AngleAxisd(frame_yaw + this->swing_yaw_l_, Vector3d::UnitZ()).toRotationMatrix();
        this->base_yaw_ = frame_yaw + 0.5 * this->swing_yaw_l_;
        this->swing_foot_pos_ = this->oMs_.act(this->swing_pos_l_);
        this->swing_foot_vel_.noalias() = R * this->swing_vel_l_;
        this->swing_foot_acc_.noalias() = R * this->swing_acc_l_;
    }

    Vector2d WalkPlanner::getRefZMP() const
    {
        const Vector2d &zmp = this->lip_.getRefZMP();
        return this->oMs_.act(Vector3d(zmp.x(), zmp.y(), 0.0)).head<2>();
    }

    Vector2d WalkPlanner::getMeasuredCP() const
    {
        return this->oMs_.act(Vector3d(this->cp_meas_.x(), this->cp_meas_.y(), 0.0)).head<2>();
    }

    Vector2d WalkPlanner::getRefCP() const
    {
        const Vector2d &cp = this->lip_.getRefCP();
        return this->oMs_.act(Vector3d(cp.x(), cp.y(), 0.0)).head<2>();
    }
}
