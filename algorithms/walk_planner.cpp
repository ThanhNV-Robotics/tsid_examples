#include "walk_planner.h"
#include <algorithm>
#include "data_type.h"
#include <cmath>

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

WalkPlanner::WalkPlanner(const double dtIn, const double zIn, double wd_hipIn)
{
    this->dt_ = dtIn;
    this->zc_ = zIn;
    this->wd_hip = wd_hipIn;
    this->w = std::sqrt(this->g/this->zc_);
    xc_ = 0; yc_= 0;
    d_xc_ = d_yc_ = 0;
    cxi_x_ = cxi_y_ = 0;
    cxi_xd_ = cxi_yd_ = 0;
    cxi_x0_ = cxi_y0_ = 0;
    leg_state_ = LegState::DSt; // init at double stand
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

void WalkPlanner::planWalking (MyGaitScheduler &gait_scheduler, JoyStickInterpreter &joyStick)
{
    // gait_scheduler: provide the gait phase variable
    // joyStick: provide the walking velocity

    double vx = joyStick.vx_W; // forward walking velocity
    t_swing = gait_scheduler.tSwing;
    // The CoM reference advances step_length per swing, so the CoM moves at
    // step_length / t_swing = vx. Must match FootPlacement::planFootstep(),
    // which lands each foot stepLength = vx * tSwing ahead of the stance foot.
    this->step_length = std::clamp(vx * t_swing, -max_step_length, max_step_length);
    auto phi = gait_scheduler.phi; // phase variable

    if (gait_scheduler.legState != leg_state_ &&
        (gait_scheduler.legState == LegState::LSt || gait_scheduler.legState == LegState::RSt))
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ += step_length; // advance CoM reference by one step
        cxi_yd_ = yBias + sideSign(gait_scheduler.legState) * 0.5 * wd_hip;
    }

    // Compute desired ZMP -- boundary-value blend that drives the capture
    // point to arrive exactly at cxi_xd_/cxi_yd_ by phi==1 (end of phase).
    const double e = 2.718281828459;
    if (phi < 1.0)
    {
        const double b = std::pow(e, this->w * this->t_swing);
        zmp_x_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
        zmp_y_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);
    }
    else
    {
        zmp_x_d_ = cxi_xd_;
        zmp_y_d_ = cxi_yd_;
    }

    // compute Capture Point
    this->computeCP(zmp_x_d_, zmp_y_d_);

    // calculate CoM
    this->computeCoM(cxi_x_, cxi_y_);

    if (gait_scheduler.legState != this->leg_state_)
        leg_state_swing_ = oppositeLeg(this->leg_state_);
    phi_CoM = phi;
    this->leg_state_ = gait_scheduler.legState;

    return;
}

