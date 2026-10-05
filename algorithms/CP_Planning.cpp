#include "CP_Planning.h"
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
// leg_state_swing_'s comment in CP_Planning.h.
LegState oppositeLeg(LegState s)
{
    if (s == LegState::RSt) return LegState::LSt;
    if (s == LegState::LSt) return LegState::RSt;
    return LegState::DSt;
}
} // namespace

CP_Planning::CP_Planning(const double dtIn, const double zIn, double wd_hipIn)
{
    this->dt_ = dtIn;
    this->zc_ = zIn;
    this->wd_hip = this->swayAmplitudeScale*wd_hipIn;
    this->w = std::sqrt(this->g/this->zc_);
    this->phi_swing = 0.0;
    xc_ = 0; yc_= 0;
    d_xc_ = d_yc_ = 0;
    cxi_x_ = cxi_y_ = 0;
    cxi_xd_ = cxi_yd_ = 0;
    cxi_x0_ = cxi_y0_ = 0;
    leg_state_ = LegState::DSt; // init at double stand

}

void CP_Planning::setInitCom (Vector3d com_pos)
{
    this->xc_ = com_pos[0];
    this->yc_ = com_pos[1];
    this->zc_ = com_pos[2];

    this->xBias = com_pos[0];
    this->yBias = com_pos[1];
}

Vector3d CP_Planning::getCoMref()
{
    Vector3d comRef{xc_, yc_, zc_};
    return comRef;
}

double CP_Planning::CoM_dynamics(double cxi, double xc)
{
    // Input: cxi: capture point input, xc: CoM position
    double d_xc;
    d_xc = this->w* (cxi - xc);
    return d_xc;
}

double CP_Planning::CP_dynamics (double p, double cxi)
{
    double d_cxi;
    d_cxi = this->w*(cxi - p);
    return  d_cxi;
}

void CP_Planning::computeCoM(double cxi_x, double cxi_y)
{
    d_xc_ = CoM_dynamics(cxi_x, this->xc_);
    d_yc_ = CoM_dynamics(cxi_y, this->yc_);
    xc_ += d_xc_*dt_;
    yc_ += d_yc_*dt_;
}

void CP_Planning::computeCP (double zmp_x, double zmp_y)
{
    double d_cxi_x{0}, d_cxi_y{0};
    d_cxi_x = CP_dynamics(zmp_x, this->cxi_x_);
    d_cxi_y = CP_dynamics(zmp_y, this->cxi_y_);

    this->cxi_x_ += d_cxi_x*dt_;
    this->cxi_y_ += d_cxi_y*dt_;
}

void CP_Planning::planSwaySin(MyGaitScheduler &gait_scheduler, double centerY)
{
    // No capture-point ODE, no leg_state_ transition logic -- just a sine
    // wave in Y around centerY, whose phase tracks gait_scheduler.phi
    // (0->1 per gait phase) instead of an internally-accumulated clock.
    // xc_/d_xc_ are untouched (whatever the caller seeded them to).
    const double phi = gait_scheduler.phi;
    const double omega = 2.0 * 3.14159265358979 * swayCyclesPerPhase;
    yc_ = centerY + swayAmplitude * std::sin(omega * phi);
    // d(sin(omega*phi))/dt = omega*cos(omega*phi)*dphi/dt. Assumes dphi/dt
    // == 1/tSwing unconditionally (the rate MyGaitScheduler::step() itself
    // uses for WARM_UP/WALK) -- it's the CALLER's job to only be advancing
    // phi at that same rate when this is called, whether via a real
    // gait_scheduler.step() or driving phi by hand (e.g. to keep motionState
    // pinned at STAND while still using phi as a time base).
    const double dphidt = (gait_scheduler.tSwing < 1e-6) ? 0.0 : (1.0 / gait_scheduler.tSwing);
    d_yc_ = swayAmplitude * omega * std::cos(omega * phi) * dphidt;
}



void CP_Planning::planWalking (MyGaitScheduler &gait_scheduler, JoyStickInterpreter &joyStick)
{
    // gait_scheduler: provide the gait phase variable
    // joyStick: provide the walking velocity

    double vx = joyStick.vx_W; // forward walking velocity
    t_swing = gait_scheduler.tSwing;
    // step_length must use the same formula as the Raibert foot placement heuristic
    // (foot_placement.cpp: posDes_W = hipPos_W + 0.5*T*v_des + ...) so that the
    // CP CoM reference advances by the same amount per step as the foot lands forward.
    // Using vx/T here instead caused a 0.1 m/step CoM-vs-foot mismatch that accumulated.
    this->step_length = 0.5 * t_swing * vx;
    auto phi = gait_scheduler.phi; // phase variable

    if (this->step_length >= 0.1) // saturation
    {
        this->step_length = 0.1;
    }


    // On every leg-state transition (edge-triggered: leg_state_ only differs
    // from gait_scheduler.legState on the one tick the transition happens),
    // re-anchor the boundary-value blend at the CP's current position and
    // aim it at the new stance side, advancing the forward reference by one
    // step. Target depends only on the NEW state, not the old one -- DSt is
    // never a transition target here (only the initial state), so only
    // LSt/RSt need handling. cxi_yd_'s sign matches the original,
    // validated-correct convention (see git history / HEAD's version of
    // this file) -- a session-internal edit briefly flipped these signs
    // based on a mistaken "previously inverted" assumption, which actually
    // made the CoM shift toward the SWINGING leg's side instead of the
    // stance leg's (confirmed by observed runtime behavior: the foot that
    // lifts is the one the CoM was shifting toward, i.e. exactly backwards
    // and unsupported). Reverted back to match HEAD.
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
        px_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
        py_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);
    }
    else
    {
        px_d_ = cxi_xd_;
        py_d_ = cxi_yd_;
    }

    // compute Capture Point
    this->computeCP(px_d_, py_d_);

    // calculate CoM
    this->computeCoM(cxi_x_, cxi_y_);

    // NOT delayed like planWarmingUp() -- the 1-cycle swing delay only
    // solves warm-up's specific problem (lifting before ANY weight has
    // shifted off a static double-support start, with no momentum to carry
    // the transfer). Steady walking has no such problem: cxi_xd_ advances by
    // step_length every cycle regardless of delay, so pairing the swing with
    // a one-cycle-STALE leg_state_ means each foot only gets to step every
    // OTHER cycle while the CoM reference advances every cycle -- the CoM
    // reference permanently outruns the feet by a growing gap ("base moving
    // faster than the foot step"). Kept synchronous: FootPlacement should
    // read leg_state_ (via the gait_scheduler-driven updateFromRobot
    // overload) for this path, not leg_state_swing_/phi_swing.
    this->leg_state_ = gait_scheduler.legState;

    return;
}

void CP_Planning::planWarmingUp (MyGaitScheduler &gait_scheduler)
{

    auto phi = gait_scheduler.phi;
    t_swing = gait_scheduler.tSwing;

    // Edge-triggered, same as planWalking() -- see its comment. No forward
    // sway here (cxi_xd_ holds at xBias, warm-up never steps), and the
    // lateral target is scaled by swayAmplitudeScale for callers that want
    // a smaller in-place sway than a full weight-shift. Both offset by
    // xBias/yBias (see CP_Planning.h) -- without it these targets are 0/
    // +-0.5*wd_hip in absolute world-frame terms, pulling the CoM toward
    // world (0,0) instead of around the robot's actual stance position.
    if (gait_scheduler.legState != leg_state_ &&
        (gait_scheduler.legState == LegState::LSt || gait_scheduler.legState == LegState::RSt))
    {
        cxi_x0_ = this->cxi_x_;
        cxi_y0_ = this->cxi_y_;

        cxi_xd_ = xBias;
        cxi_yd_ = yBias + sideSign(gait_scheduler.legState) * 0.5 * wd_hip * swayAmplitudeScale;
    }

    // calculate ZMP
    const double e = 2.718281828459;
    if (phi < 1.0)
    {
        const double b = std::pow(e, this->w * this->t_swing);
        px_d_ = (cxi_xd_ - b*cxi_x0_)/(1-b);
        py_d_ = (cxi_yd_ - b*cxi_y0_)/(1-b);
    }
    else
    {
        px_d_ = cxi_xd_;
        py_d_ = cxi_yd_;
    }

    // compute Capture Point
    this->computeCP(px_d_, py_d_);

    // calculate CoM
    this->computeCoM(cxi_x_, cxi_y_);

    // Swing-foot phase: 1-cycle delay vs the CoM phase (phi). On the tick
    // leg_state_ actually transitions, the ENDING leg_state_ (the target the
    // CoM has just finished shifting onto) tells us which side now carries
    // the weight -- the foot that's now free to swing is the OPPOSITE side
    // (oppositeLeg(): leg_state_ is a TARGET label, not "which foot is
    // free", so it must be inverted before FootPlacement/KinWBC's RSt-keyed
    // stance/swing convention consumes it). phi_swing just mirrors phi --
    // both ramp 0->1 over the same t_swing, the delay lives entirely in
    // which leg_state_swing_ value phi_swing is paired with.
    if (gait_scheduler.legState != this->leg_state_)
        leg_state_swing_ = oppositeLeg(this->leg_state_);
    phi_swing = phi;

    this->leg_state_ = gait_scheduler.legState;

    return;
}

