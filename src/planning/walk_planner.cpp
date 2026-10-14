#include "planning/walk_planner.h"
#include <algorithm>
#include "common/data_type.h"
#include "planning/joystick_interpreter.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <vector>

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
    this->td_confirm_time_ = readOr(cfg, "touchdown_confirm_time", this->td_confirm_time_);
    this->late_confirm_time_ = readOr(cfg, "late_confirm_time", this->late_confirm_time_);
    this->late_timeout_ = readOr(cfg, "late_touchdown_timeout", this->late_timeout_);
    this->zmp_hold_time_ = readOr(cfg, "zmp_hold_time", this->zmp_hold_time_);
    this->cp_feedback_gain_ = readOr(cfg, "cp_feedback_gain", this->cp_feedback_gain_);
    this->t_ds_ = readOr(cfg, "double_support_time", this->t_ds_);
    this->foothold_adaptation_ = readOr(cfg, "foothold_adaptation", this->foothold_adaptation_);
    this->foothold_max_width_change_ = readOr(cfg, "foothold_max_width_change", this->foothold_max_width_change_);
    this->foothold_max_rate_ = readOr(cfg, "foothold_max_rate", this->foothold_max_rate_);
    this->foothold_freeze_phase_ = readOr(cfg, "foothold_freeze_phase", this->foothold_freeze_phase_);

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
                tsid::robots::RobotWrapper &robot_wrapper, const pinocchio::Data &data, double Tswing,
                const std::vector<bool> &contact_flags)
{
    // Planning CoM and swing feet using local coordinate to avoid driting
    this->stance_changed_ = false; // set again by enterStance() if a new step starts this tick
    this->liftoff_event_ = false;  // set again when the double support ends this tick
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
    
    const double dPhi = dt_ / Tswing; // phase goes 0 -> 1 over one Tswing
    // During the double support of a step the single-support phase waits
    if (in_ds_) this->ds_time_ += dt_;
    else this->phi_CoM += dPhi; // ramping CoM phase
    

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
        if (stance_leg_ == LegState::DSt) 
        {
            enterStance(LegState::LSt, robot_wrapper, data); // left leg stand first (first step)
        } 
        
        else if (in_ds_) // double support: both feet loaded, no swing foot
        {
            if (ds_time_ >= t_ds_) {
                in_ds_ = false;          // the trailing foot lifts off, single support starts
                liftoff_event_ = true;
            }
        }
        else // during walking        
        {
            // which leg is in stand: true = left stand/right swing, false = right stand/left swing
            const bool left_stance = (stance_leg_ == LegState::LSt); 

            // contact flags (from fusion model or cheat estimator)
            // contact flags is [left, right]
            // the planner determin which leg is swinging
            // if left_stance then take contact_flags[1] -> right foot swings
            const bool swing_contact = contact_flags[left_stance ? 1 : 0];


            if (!swing_contact) swing_lifted_ = true;
            // Time the swing foot has been back in contact without interruption
            swing_contact_time_ = (swing_lifted_ && swing_contact) ? swing_contact_time_ + dt_ : 0.0;

            // Early touchdown: the swing foot lifted and is back on the ground,
            // confirmed for td_confirm_time_ (a light touch just before phi = 1
            // or a bounce does not end the step), late enough in the step
            // (rejects flicker right after lift-off).
            const bool touchdown = swing_lifted_ && swing_contact_time_ >= td_confirm_time_
                                   && phi_CoM >= phi_td_min_;
            const bool step_end = phi_CoM >= 1.0;

            // Late touchdown: the step is over but the swing foot is not on the
            // ground (ground lower than expected). Ending the step now would add
            // a rigid TSID contact in the air and drop the robot onto it, so the
            // support leg is kept while the swing foot keeps descending; the step
            // ends on a confirmed contact, or after late_timeout_ (the CP
            // diverges meanwhile: constant ZMP on the old stance foot)

            // if the step ends but the swing foot not contact yet
            if (step_end && !late_ && !swing_contact) {
                late_ = true; // turn on this flag to indicate that late contact dectected
                late_time_ = 0.0;
            }

            // // Phase 2 handle the late touch
            bool end_step = touchdown || step_end;
            if (late_) {
                late_time_ += dt_;
                const bool late_touchdown = swing_contact_time_ >= late_confirm_time_;
                // The foot keeps stretching down until the touchdown; the timeout
                // is an optional safety net (late_timeout_ <= 0: wait for contact)
                end_step = late_touchdown || (late_timeout_ > 0.0 && late_time_ >= late_timeout_);
                if (end_step && !late_touchdown) {
                    std::printf("[WalkPlanner] late touchdown timeout (%.3f s): step ended without contact\n",
                                late_time_);
                }
            }
            if (end_step) {
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
        const Eigen::Vector2d foot_nominal = R_err * Eigen::Vector2d(step_length, side * wd_hip);
        // Each step starts from the nominal foothold; Stage 3 adapts it below
        if (stance_changed_ || !foothold_adaptation_) this->next_foot_pos_ = foot_nominal;

        // Nominal CP targets of the periodic gait (Stage 3b): ZMP constant at the
        // stance ankle during single support (SS) and at the midpoint between the
        // feet during double support (DS). With E_ss = e^(w T_ss), E_ds = e^(w T_ds),
        // the CP at the start of SS, relative to the stance foot, is
        //   xi0 = ( s (1 + E_ds) / (2 (E_ds E_ss - 1)),  side w_h (1 + E_ds) / (2 (1 + E_ds E_ss)) )
        // and at the end of SS (touchdown) xi_ss_end = E_ss xi0. For T_ds = 0 they
        // are the single-support targets s / (b - 1), y_f / (b + 1) and b xi0.
        const double b = std::exp(this->w * this->t_swing);       // E_ss
        const double e_ds = std::exp(this->w * this->t_ds_);      // E_ds
        const Eigen::Vector2d cxi_start = R_err * Eigen::Vector2d(
            step_length * (1.0 + e_ds) / (2.0 * (e_ds * b - 1.0)),
            side * wd_hip * (1.0 + e_ds) / (2.0 * (1.0 + e_ds * b)));
        const Eigen::Vector2d cxi_ss_end = b * cxi_start;
        // Active target: start of SS while in DS, touchdown CP while in SS
        const Eigen::Vector2d cxi_d = in_ds_ ? cxi_start : cxi_ss_end;
        this->cxi_xd_ = cxi_d.x();
        this->cxi_yd_ = cxi_d.y();

        // Compute ZMP and CoM reference w.r.t the ankle frame
        // Receding-horizon ZMP (doc/event_based_control.md, Stage 1): every tick,
        // the constant ZMP that takes the CP from its current value to cxi_d in
        // the remaining step time, clamped to the sole. Undisturbed, this equals
        // the once-per-step solution; a CP error (clamping, a step that started
        // early or late) is spread over the rest of the step instead of being
        // frozen at the step start. The solution is ill-conditioned when the
        // remaining time goes to 0, so the ZMP is held for the last
        // zmp_hold_time_ (and in the late state, phi > 1).
        //
        // Stage 2: the CP in that formula blends the measured CP of the robot
        // into the planned one, so the ZMP reacts when the robot lags the plan
        // (DCM feedback). The planned CP and CoM keep being integrated from the
        // commanded ZMP and give TSID a smooth CoM reference.
        measureCP(robot_wrapper, data);
        const double cxi_fb_x = cxi_x_ + cp_feedback_gain_ * (cxi_meas_.x() - cxi_x_);
        const double cxi_fb_y = cxi_y_ + cp_feedback_gain_ * (cxi_meas_.y() - cxi_y_);
        // Remaining time of the current phase (DS: to liftoff, SS: to touchdown)
        const double t_rem = in_ds_ ? std::max(t_ds_ - ds_time_, 0.0)
                                    : this->t_swing * (1.0 - std::min(phi_CoM, 1.0));
        if (t_rem > zmp_hold_time_) {
            const double b_rem = std::exp(this->w * t_rem);
            const Eigen::Vector2d p((cxi_xd_ - b_rem * cxi_fb_x) / (1.0 - b_rem),
                                    (cxi_yd_ - b_rem * cxi_fb_y) / (1.0 - b_rem));
            const Eigen::Vector2d p_c = in_ds_ ? clampToDoubleSupport(p) : clampToSole(p);
            zmp_x_d_ = p_c.x();
            zmp_y_d_ = p_c.y();
        }

        // Stage 3: CP-based foothold adaptation. Predicted CP at the end of the
        // step with the commanded (clamped) ZMP over the remaining time, and the
        // foothold from which the next step starts like the periodic gait:
        //   cxi_end = p + e^(w t_rem) (cxi - p),  p_next = cxi_end - cxi0_next
        //   cxi0_next = (s / (b - 1), -side w_h / (b + 1)) relative to the next foot
        // When the ZMP is not clamped, cxi_end = cxi_d and p_next is the nominal
        // foothold; when it is clamped, the foothold takes over the part of the
        // correction the ZMP cannot do. e^(w t_rem) is large early in the step,
        // so the foothold is clamped, rate limited and frozen late in the swing.
        // (Stage 3b: the touchdown CP relative to the next foot should be the
        // nominal one, cxi_ss_end - foot_nominal, from which the next double
        // support starts; for T_ds = 0 this is the formula above.)
        if (foothold_adaptation_ && !in_ds_ && std::min(phi_CoM, 1.0) <= foothold_freeze_phase_) {
            const Eigen::Vector2d zmp(zmp_x_d_, zmp_y_d_);
            const Eigen::Vector2d cxi_fb(cxi_fb_x, cxi_fb_y);
            const Eigen::Vector2d cxi_end = zmp + std::exp(this->w * t_rem) * (cxi_fb - zmp);
            const Eigen::Vector2d cxi_td_rel_next = cxi_ss_end - foot_nominal;
            // Limits in the walking-heading axes: step length, lateral range
            // around the nominal width (keeps the feet apart, no crossing)
            Eigen::Vector2d q = R_err.inverse() * (cxi_end - cxi_td_rel_next);
            q.x() = std::clamp(q.x(), -max_step_length, max_step_length);
            q.y() = std::clamp(q.y(), side * wd_hip - foothold_max_width_change_,
                               side * wd_hip + foothold_max_width_change_);
            // Rate limit, so the swing target moves smoothly
            const Eigen::Vector2d step = R_err * q - next_foot_pos_;
            const double max_step = foothold_max_rate_ * dt_;
            next_foot_pos_ += (step.norm() > max_step) ? Eigen::Vector2d(step * (max_step / step.norm())) : step;
        }

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
    reanchor(oMst.translation().head<2>(), st_yaw);

    // Stage 2: at every touchdown pull the planned CoM and CP toward the
    // measured ones, so the plan cannot drift away from the robot over many
    // steps (the reference stays continuous for cp_feedback_gain_ < 1)
    measureCP(robot_wrapper, data);
    this->xc_ += cp_feedback_gain_ * (com_meas_.x() - this->xc_);
    this->yc_ += cp_feedback_gain_ * (com_meas_.y() - this->yc_);
    this->cxi_x_ += cp_feedback_gain_ * (cxi_meas_.x() - this->cxi_x_);
    this->cxi_y_ += cp_feedback_gain_ * (cxi_meas_.y() - this->cxi_y_);

    // Trailing (previous stance) foot, in the new planning frame: the other
    // edge of the double-support region
    const std::string &trail_frame = (stance == LegState::LSt) ? right_foot_frame : left_foot_frame;
    this->trail_foot_ = worldToPoint(
        robot_wrapper.framePosition(data, robot_wrapper.model().getFrameId(trail_frame)).translation().head<2>());
    // The step starts with the double support (Stage 3b); without it the
    // trailing foot lifts off on this tick
    this->in_ds_ = (t_ds_ > 0.0);
    this->ds_time_ = 0.0;
    if (!in_ds_) this->liftoff_event_ = true;

    this->stance_leg_ = stance;
    this->cxi_x0_ = this->cxi_x_; // start of the boundary-value phase
    this->cxi_y0_ = this->cxi_y_;
    this->phi_CoM = 0.0;
    this->swing_lifted_ = false;
    this->swing_contact_time_ = 0.0;
    this->late_ = false;
    this->late_time_ = 0.0;
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

Eigen::Vector2d WalkPlanner::worldToPoint(const Eigen::Vector2d &p_W) const
{
    return Eigen::Rotation2Dd(-frame_yaw_) * (p_W - frame_origin_W_);
}

Eigen::Vector2d WalkPlanner::worldToDir(const Eigen::Vector2d &v_W) const
{
    return Eigen::Rotation2Dd(-frame_yaw_) * v_W;
}

Eigen::Vector2d WalkPlanner::clampToSole(const Eigen::Vector2d &p) const
{
    return Eigen::Vector2d(std::clamp(p.x(), sole_x_min_, sole_x_max_),
                           std::clamp(p.y(), -sole_y_half_, sole_y_half_));
}

Eigen::Vector2d WalkPlanner::clampToDoubleSupport(const Eigen::Vector2d &p) const
{
    // Band from the stance ankle (planning-frame origin) to the trailing ankle:
    // along the band, both ankles plus the sole half width; across it, the
    // sole's toe/heel extent (the band runs mostly sideways, across = forward)
    const Eigen::Vector2d ab = trail_foot_;
    const double len = ab.norm();
    if (len < 1e-6) return clampToSole(p);
    const Eigen::Vector2d u = ab / len;
    Eigen::Vector2d n(-u.y(), u.x());
    if (n.x() < 0.0) n = -n; // across axis pointing forward
    const double along = std::clamp(p.dot(u), -sole_y_half_, len + sole_y_half_);
    const double across = std::clamp(p.dot(n), sole_x_min_, sole_x_max_);
    return along * u + across * n;
}

void WalkPlanner::measureCP(const tsid::robots::RobotWrapper &robot_wrapper, const pinocchio::Data &data)
{
    // CoM position and velocity of the robot (computeAllTerms of the last TSID
    // problem), in the planning frame; CP of the LIPM: c + dc / w
    com_meas_ = worldToPoint(robot_wrapper.com(data).head<2>());
    const Eigen::Vector2d vcom = worldToDir(robot_wrapper.com_vel(data).head<2>());
    cxi_meas_ = com_meas_ + vcom / this->w;
}

Vector3d WalkPlanner::getMeasuredCPW() const
{
    const Eigen::Vector2d p = pointToWorld(cxi_meas_.x(), cxi_meas_.y());
    return Vector3d(p.x(), p.y(), 0.0);
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
