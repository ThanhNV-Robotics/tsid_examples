#include "walk_planner.h"
#include <algorithm>
#include "data_type.h"
#include "joystick_interpreter.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <vector>

namespace
{
// +1 for the right-stance target side, -1 for left, 0 (no target) for DSt.
// Matches the original, validated-correct convention: entering LSt aims the
// CoM at -0.5*wd_hip, entering RSt at +0.5*wd_hip (see git history / HEAD).
double sideSign(LegState s)
{
    if (s == LegState::RSt) return 1.0;
    if (s == LegState::LSt) return -1.0;
    return 0.0;
}

// LSt<->RSt swapped, DSt unchanged. Used to turn a TARGET label (which side
// the CoM was shifting toward) into "which foot is now free" -- see
// leg_state_swing_'s comment in WalkPlanner.h.
LegState oppositeLeg(LegState s)
{
    if (s == LegState::RSt) return LegState::LSt;
    if (s == LegState::LSt) return LegState::RSt;
    return LegState::DSt;
}
} // namespace

namespace
{
// "walk_planner: foot_placement:" block, or an empty node if it is missing
YAML::Node loadWalkPlannerConfig(const std::string &config_path)
{
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node wp = root["walk_planner"];
    const YAML::Node fp = wp ? wp["foot_placement"] : YAML::Node();
    if (!fp) {
        std::cout << "[WalkPlanner] No 'walk_planner: foot_placement:' block in " << config_path
                  << ", using defaults" << std::endl;
    }
    return fp;
}

template <typename T>
T readOr(const YAML::Node &node, const char *key, const T &fallback)
{
    return (node && node[key]) ? node[key].as<T>() : fallback;
}
} // namespace

WalkPlanner::WalkPlanner(const std::string &config_path, const double zIn)
    : WalkPlanner(loadWalkPlannerConfig(config_path), zIn)
{
}

WalkPlanner::WalkPlanner(const YAML::Node &cfg, const double zIn)
    : left_foot_frame(readOr<std::string>(cfg, "left_foot_frame", "left_ankle_pitch_link")),
      right_foot_frame(readOr<std::string>(cfg, "right_foot_frame", "right_ankle_pitch_link"))
{
    this->dt_ = readOr(cfg, "dt", 0.001);
    this->wd_hip = readOr(cfg, "stance_width", this->wd_hip);
    this->max_step_length = readOr(cfg, "max_step_length", this->max_step_length);
    this->sole_x_min_ = readOr(cfg, "sole_x_min", this->sole_x_min_);
    this->sole_x_max_ = readOr(cfg, "sole_x_max", this->sole_x_max_);
    this->sole_y_half_ = readOr(cfg, "sole_y_half", this->sole_y_half_);
    this->phi_td_min_ = readOr(cfg, "touchdown_phase_min", this->phi_td_min_);

    this->zc_ = zIn;
    this->w = std::sqrt(this->g/this->zc_);
    xc_ = 0; yc_= 0;
    d_xc_ = d_yc_ = 0;
    cxi_x_ = cxi_y_ = 0;
    cxi_xd_ = cxi_yd_ = 0;
    cxi_x0_ = cxi_y0_ = 0;
}

void WalkPlanner::setInitCom (Vector3d com_pos, Vector3d com_vel)
{
    this->xc_ = com_pos[0];
    this->yc_ = com_pos[1];
    this->zc_ = com_pos[2];
    this->w = std::sqrt(this->g / this->zc_); // keep w consistent with the new height

    this->d_xc_ = com_vel[0];
    this->d_yc_ = com_vel[1];

    // Capture point consistent with the CoM state; without this it stays at
    // (0, 0) from the constructor and d_xc = w(cxi - xc) drags the CoM there
    this->cxi_x_ = this->xc_ + this->d_xc_ / this->w;
    this->cxi_y_ = this->yc_ + this->d_yc_ / this->w;
    this->cxi_x0_ = this->cxi_xd_ = this->cxi_x_;
    this->cxi_y0_ = this->cxi_yd_ = this->cxi_y_;
    this->zmp_x_d_ = this->cxi_x_;
    this->zmp_y_d_ = this->cxi_y_;

    this->xBias = com_pos[0];
    this->yBias = com_pos[1];
}

Vector3d WalkPlanner::getCoMref()
{
    Vector3d comRef{xc_, yc_, zc_};
    return comRef;
}

Vector3d WalkPlanner::getCoMvelRef() const
{
    return Vector3d(d_xc_, d_yc_, 0.0);
}

Vector3d WalkPlanner::getCoMaccRef() const
{
    return Vector3d(w * w * (xc_ - zmp_x_d_), w * w * (yc_ - zmp_y_d_), 0.0);
}

double WalkPlanner::CoM_dynamics(double cxi, double xc)
{
    // Input: cxi: capture point input, xc: CoM position
    double d_xc;
    d_xc = this->w* (cxi - xc);
    return d_xc;
}

double WalkPlanner::CP_dynamics (double p, double cxi)
{
    double d_cxi;
    d_cxi = this->w*(cxi - p);
    return  d_cxi;
}

void WalkPlanner::computeCoM(double cxi_x, double cxi_y)
{
    d_xc_ = CoM_dynamics(cxi_x, this->xc_);
    d_yc_ = CoM_dynamics(cxi_y, this->yc_);
    xc_ += d_xc_*dt_;
    yc_ += d_yc_*dt_;
}

void WalkPlanner::computeCP (double zmp_x, double zmp_y)
{
    double d_cxi_x{0}, d_cxi_y{0};
    d_cxi_x = CP_dynamics(zmp_x, this->cxi_x_);
    d_cxi_y = CP_dynamics(zmp_y, this->cxi_y_);

    this->cxi_x_ += d_cxi_x*dt_;
    this->cxi_y_ += d_cxi_y*dt_;
}

void WalkPlanner::planWalking (JoyStickInterpreter &joystick, Cheat_StateEstimator &state_estimator,
                tsid::robots::RobotWrapper &robot_wrapper, const pinocchio::Data &data, double Tswing)
{
    // Planning CoM and swing feet using local coordinate to avoid driting
    this->stance_changed_ = false; // set again by enterStance() if a new step starts this tick
    const double e = 2.718281828459;
    if (!isInit_) // if not init yet
    {
        // init once
        // capture current robot yaw angle/direction (heading)
        // walking command will stablize to remain at this heading direction
        const Eigen::Matrix3d R = state_estimator.getState().quat_b_W.toRotationMatrix();
        this->ref_theta = std::atan2(R(1, 0), R(0, 0));
        this->isInit_ = true;

        // capture current CoM pos and velocity w.r.t origin at the pelvis center
        const auto &st = state_estimator.getState();
        const Vector3d com_W  = robot_wrapper.com(data);      // CoM position, world frame
        const Vector3d vcom_W = robot_wrapper.com_vel(data);  // CoM velocity, world frame

        // heading frame: origin at the pelvis, rotated by yaw only (stays level)
        const Eigen::Matrix3d Rz = Eigen::AngleAxisd(ref_theta, Vector3d::UnitZ()).toRotationMatrix();
        Vector3d com_P  = Rz.transpose() * (com_W - st.pos_b_W);
        const Vector3d vcom_P = Rz.transpose() * vcom_W;
        // z must stay the CoM height above the ground (flat floor at z = 0):
        // setInitCom() derives the LIPM frequency w = sqrt(g / z) from it, and the
        // pelvis-relative z is ~0 or negative (w huge or NaN)
        com_P[2] = com_W[2];

        setInitCom(com_P, vcom_P);
        this->frame_origin_W_ = st.pos_b_W.head<2>();
        this->frame_yaw_ = ref_theta;
    }

    double vx = joystick.vx_W; // forward walking velocity
    t_swing = Tswing;
    std::vector<bool> contact_flags = state_estimator.getState().contact_flags;
    
    const double dPhi = dt_ / Tswing; // phase goes 0 -> 1 over one Tswing
    this->phi_CoM += dPhi; // ramping CoM phase
    

    this->step_length = std::clamp(vx * t_swing, -max_step_length, max_step_length);

    // double support case: init start walking
    // both foot in contact and not done init stepping
    // this to make the swing phase of swinging leg to be 1 cycle delay compare to CoM swing phase
    // CoM swing to the left leg side first to move the ZMP to left sole
    if (contact_flags[0] && contact_flags[1] && !initStepping) 
    {
        // printf("prepare stepping\n");
        cxi_xd_ = this->xBias; 
        cxi_yd_ = yBias + 0.5 * wd_hip; // swing CoM to left feet first / left leg stand first

        // Compute desired ZMP -- boundary-value blend that drives the capture
        // point to arrive exactly at cxi_xd_/cxi_yd_ by phi==1 (end of phase).
        

        if (phi_CoM >= 1)
        {
            zmp_x_d_ = cxi_xd_;
            zmp_y_d_ = cxi_yd_;
            initStepping = true; // done moving the ZMP to the supporting leg, swinging leg can start
        }

        else
        {
            const double b = std::pow(e, this->w * this->t_swing);
            zmp_x_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
            zmp_y_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);
        }

        // compute Capture Point
        this->computeCP(zmp_x_d_, zmp_y_d_);

        // calculate CoM
        this->computeCoM(cxi_x_, cxi_y_);
    }

    // Single support stepping: left leg stands first (the init shift moved the
    // CP onto the left foot). The stance leg is the planner's decision; contact
    // flags are only used to detect the swing foot's touchdown.
    if (initStepping)
    {
        if (stance_leg_ == LegState::DSt) {
            enterStance(LegState::LSt, robot_wrapper, data);
        } else {
            const bool left_stance = (stance_leg_ == LegState::LSt);
            const bool swing_contact = contact_flags[left_stance ? 1 : 0];
            if (!swing_contact) swing_lifted_ = true;

            // Early touchdown: the swing foot lifted and is back on the ground
            // late enough in the step (rejects flicker right after lift-off).
            // Otherwise the step always ends on time: the swing trajectory has
            // brought the foot down by phi = 1, and waiting for the contact flag
            // instead lets the CP diverge (constant ZMP on the old stance foot)
            const bool touchdown = swing_lifted_ && swing_contact && phi_CoM >= phi_td_min_;
            const bool step_end = phi_CoM >= 1.0;
            if (touchdown || step_end) {
                enterStance(left_stance ? LegState::RSt : LegState::LSt, robot_wrapper, data);
            }
        }

        // Origin at the stance ankle, x along the stance foot's yaw
        const bool left_stance = (stance_leg_ == LegState::LSt);
        const double side = left_stance ? -1.0 : 1.0; // swing foot lands to the right of a left stance

        // swing phase: stance foot 0, swing foot ramps with the step
        this->phi_Lfeet = left_stance ? 0.0 : std::min(phi_CoM, 1.0);
        this->phi_Rfeet = left_stance ? std::min(phi_CoM, 1.0) : 0.0;

        // wrapped to [-pi, pi] so headings near +-pi don't give a ~2pi error
        const double heading_error = std::remainder(frame_yaw_ - this->ref_theta, 2.0 * M_PI);

        // compute next desired foot hole which compensate for the heading error also
        // The step is defined along the reference heading (x = ref_theta, y = left),
        // then rotated into the stance-foot axes by -heading_error, so the swing
        // foot lands along ref_theta even if the stance foot is yawed
        const Eigen::Rotation2Dd R_err(-heading_error);
        this->next_foot_pos_ = R_err * Eigen::Vector2d(step_length, side * wd_hip);

        // End-of-step capture point for a constant ZMP at the stance ankle: the
        // periodic LIPM solution, so the ZMP stays at the ankle in steady walking
        //   sagittal: xi_d = s * b / (b - 1)      lateral: xi_d = y_f * b / (b + 1)
        const double b = std::exp(this->w * this->t_swing);
        const Eigen::Vector2d cxi_d = R_err * Eigen::Vector2d(step_length * b / (b - 1.0),
                                                              side * wd_hip * b / (b + 1.0));
        this->cxi_xd_ = cxi_d.x();
        this->cxi_yd_ = cxi_d.y();

        // Compute ZMP and CoM reference w.r.t the ankle frame
        // Constant ZMP over the step that takes the CP from cxi0 (captured on
        // entering stance) to cxi_d at the end of the step, then clamped to the
        // sole: the CP may then miss cxi_d a bit, the next foothold absorbs it
        zmp_x_d_ = std::clamp((cxi_xd_ - b * cxi_x0_) / (1.0 - b), sole_x_min_, sole_x_max_);
        zmp_y_d_ = std::clamp((cxi_yd_ - b * cxi_y0_) / (1.0 - b), -sole_y_half_, sole_y_half_);

        // compute Capture Point
        this->computeCP(zmp_x_d_, zmp_y_d_);

        // calculate CoM
        this->computeCoM(cxi_x_, cxi_y_);
    }
    return;
}

void WalkPlanner::enterStance(LegState stance, const tsid::robots::RobotWrapper &robot_wrapper,
                              const pinocchio::Data &data)
{
    const std::string &frame = (stance == LegState::LSt) ? left_foot_frame : right_foot_frame;
    if (!robot_wrapper.model().existFrame(frame)) {
        throw std::runtime_error("[WalkPlanner] Frame '" + frame + "' not found in the robot model");
    }
    const pinocchio::SE3 oMst = robot_wrapper.framePosition(data, robot_wrapper.model().getFrameId(frame));
    const Eigen::Matrix3d &R_st = oMst.rotation();
    const double st_yaw = std::atan2(R_st(1, 0), R_st(0, 0)); // heading of the stance foot

    // Reset the CoM and Capture point to the stance ankle frame, frozen for the step
    reanchor(oMst.translation().head<2>(), 0*st_yaw);
    this->stance_leg_ = stance;
    this->cxi_x0_ = this->cxi_x_; // start of the boundary-value phase
    this->cxi_y0_ = this->cxi_y_;
    this->phi_CoM = 0.0;
    this->swing_lifted_ = false;
    this->stance_z_W_ = oMst.translation().z();
    this->stance_changed_ = true;
}

void WalkPlanner::reanchor(const Eigen::Vector2d &o_new_W, double yaw_new)
{
    // positions:  x_new = R_new^T (o_old + R_old x_old - o_new)
    // velocities: v_new = R_new^T R_old v_old
    const Eigen::Rotation2Dd R_old(frame_yaw_), R_new(yaw_new);
    const Eigen::Rotation2Dd R_rel = R_new.inverse() * R_old;
    const Eigen::Vector2d offset = R_new.inverse() * (frame_origin_W_ - o_new_W);

    const auto toNew = [&](double &x, double &y) {
        const Eigen::Vector2d p = R_rel * Eigen::Vector2d(x, y) + offset;
        x = p.x(); y = p.y();
    };
    toNew(xc_, yc_);
    toNew(cxi_x_, cxi_y_);
    toNew(zmp_x_d_, zmp_y_d_);

    const Eigen::Vector2d v = R_rel * Eigen::Vector2d(d_xc_, d_yc_);
    d_xc_ = v.x(); d_yc_ = v.y();

    frame_origin_W_ = o_new_W;
    frame_yaw_ = yaw_new;
}

Eigen::Vector2d WalkPlanner::pointToWorld(double x, double y) const
{
    return frame_origin_W_ + Eigen::Rotation2Dd(frame_yaw_) * Eigen::Vector2d(x, y);
}

Eigen::Vector2d WalkPlanner::dirToWorld(double x, double y) const
{
    return Eigen::Rotation2Dd(frame_yaw_) * Eigen::Vector2d(x, y);
}

Vector3d WalkPlanner::getCoMrefW() const
{
    const Eigen::Vector2d p = pointToWorld(xc_, yc_);
    return Vector3d(p.x(), p.y(), zc_);
}

Vector3d WalkPlanner::getCoMvelRefW() const
{
    const Eigen::Vector2d v = dirToWorld(d_xc_, d_yc_);
    return Vector3d(v.x(), v.y(), 0.0);
}

Vector3d WalkPlanner::getCoMaccRefW() const
{
    const Vector3d a = getCoMaccRef();
    const Eigen::Vector2d a_W = dirToWorld(a.x(), a.y());
    return Vector3d(a_W.x(), a_W.y(), 0.0);
}

Vector3d WalkPlanner::getZMPrefW() const
{
    const Eigen::Vector2d p = pointToWorld(zmp_x_d_, zmp_y_d_);
    return Vector3d(p.x(), p.y(), 0.0);
}

Vector3d WalkPlanner::getCapturedPointrefW() const
{
    const Eigen::Vector2d p = pointToWorld(cxi_x_, cxi_y_);
    return Vector3d(p.x(), p.y(), 0.0);
}

Vector3d WalkPlanner::getNextFootW() const
{
    const Eigen::Vector2d p = pointToWorld(next_foot_pos_.x(), next_foot_pos_.y());
    return Vector3d(p.x(), p.y(), stance_z_W_);
}

Vector3d WalkPlanner::getZMPref() const
{
    return Vector3d(zmp_x_d_, zmp_y_d_, 0.0);
}

Vector3d WalkPlanner::getCapturedPointref() const
{
    return Vector3d(cxi_x_, cxi_y_, 0.0);
}

Vector3d WalkPlanner::getPhase () const
{
    return Vector3d(this->phi_CoM, this->phi_Lfeet, this->phi_Rfeet);
}
