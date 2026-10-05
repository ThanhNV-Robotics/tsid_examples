/*
Adapted from OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
*/
#include "foot_placement.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace {
template <typename T>
void readIfPresent(const YAML::Node &node, const char *key, T &value)
{
    if (node && node[key]) value = node[key].as<T>();
}
} // namespace

FootPlacement::FootPlacement(const std::string &yamlPath, const tsid::robots::RobotWrapper &robot)
    : robot_(robot)
{
    // All parameters are optional; missing ones keep their defaults
    const YAML::Node root = YAML::LoadFile(yamlPath);
    const YAML::Node fp = root["foot_placement"];
    if (!fp) {
        std::cout << "[FootPlacement] No 'foot_placement' block in " << yamlPath << ", using defaults" << std::endl;
    }
    readIfPresent(fp, "stepHeight", stepHeight);
    readIfPresent(fp, "stance_width", stanceWidth);
    readIfPresent(fp, "x_offset", xOff_L);
    readIfPresent(fp, "y_offset", yOff_L);
    readIfPresent(fp, "z_offset", zOff_W);
    readIfPresent(fp, "kp_vx", kp_vx);
    readIfPresent(fp, "kp_vy", kp_vy);
    readIfPresent(fp, "kp_wz", kp_wz);
    readIfPresent(fp, "max_step_length", maxStepLength);
    readIfPresent(fp, "left_foot_frame", leftFootFrame);
    readIfPresent(fp, "right_foot_frame", rightFootFrame);

    for (const auto &frame : {leftFootFrame, rightFootFrame}) {
        if (!robot_.model().existFrame(frame)) {
            throw std::runtime_error("[FootPlacement] Frame '" + frame + "' not found in the robot model");
        }
    }
    leftFootId_ = robot_.model().getFrameId(leftFootFrame);
    rightFootId_ = robot_.model().getFrameId(rightFootFrame);
}

void FootPlacement::updateFromRobot(const RobotState &state, const pinocchio::Data &data, LegState curLegState,
                                    double phiIn, double tSwingIn, const JoyStickInterpreter &joyStick)
{
    // Snapshot the swing foot's liftoff position the instant the swing leg
    // changes; the swing trajectory starts from there. RSt -> left foot swings,
    // otherwise the right foot (DSt included, but nothing is swung in DSt).
    const bool leftSwing = (curLegState == LegState::RSt);
    if (!swingInit_ || curLegState != swingLegPrev_) {
        if (openLoopFootsteps) {
            // Chain the planned footsteps instead of reading the real feet
            if (!swingInit_) {
                plannedLeft_W = robot_.framePosition(data, leftFootId_).translation();
                plannedRight_W = robot_.framePosition(data, rightFootId_).translation();
            } else if (swingLegPrev_ != LegState::DSt) {
                // the swing that just ended landed where it was planned
                // at ground height: the z_offset stretch is only a target, the
                // foot stops on the ground
                Eigen::Vector3d landed = posDes_W;
                landed.z() = posStance_W.z();
                ((swingLegPrev_ == LegState::RSt) ? plannedLeft_W : plannedRight_W) = landed;
            }
            posStart_W = leftSwing ? plannedLeft_W : plannedRight_W;
            posStance_W = leftSwing ? plannedRight_W : plannedLeft_W;
        } else {
            posStart_W = robot_.framePosition(data, leftSwing ? leftFootId_ : rightFootId_).translation();
            posStance_W = robot_.framePosition(data, leftSwing ? rightFootId_ : leftFootId_).translation();
        }
        swingInit_ = true;
    }
    swingLegPrev_ = curLegState;
    legState = curLegState;

    phi = std::clamp(phiIn, 0.0, 1.0);
    tSwing = tSwingIn;
    theta0_ = (curLegState == LegState::RSt) ? M_PI / 2.0 : -M_PI / 2.0;

    // Base pose and velocities (RobotState velocities are in the world frame)
    base_pos = state.pos_b_W;
    const Eigen::Matrix3d R = state.quat_b_W.toRotationMatrix();
    yawCur_ = std::atan2(R(1, 0), R(0, 0));
    omegaZ_W_ = state.wb_W.z();
    curV_W = state.vb_W;

    desV_W = Eigen::Vector3d(joyStick.vx_W, joyStick.vy_W, 0.0);
    desWz_W = joyStick.wz_L; // pure z rotation: body and world rates coincide
    heading_ = joyStick.thetaZ;

    // Nominal foot position on the swing side: base position offset
    // laterally by half the stance width in the yaw-aligned frame
    const double side = (curLegState == LegState::RSt) ? 1.0 : -1.0;
    const Eigen::Vector2d lateral(-std::sin(yawCur_) * side * stanceWidth / 2.0,
                                  std::cos(yawCur_) * side * stanceWidth / 2.0);
    hipPos_W = base_pos;
    hipPos_W.head<2>() += lateral;
}

Eigen::Vector2d FootPlacement::landingOffset_W() const
{
    // yOff_L is "inward": -y for the left foot, +y for the right foot
    const double side = (legState == LegState::RSt) ? 1.0 : -1.0;
    const Eigen::Vector2d off_L(xOff_L, -side * yOff_L);
    const double c = std::cos(yawCur_), s = std::sin(yawCur_);
    return Eigen::Vector2d(c * off_L.x() - s * off_L.y(), s * off_L.x() + c * off_L.y());
}

void FootPlacement::computeSwingTrajectory()
{
    pDes_ = posStart_W;
    vDes_.setZero();
    aDes_.setZero();
    if (legState == LegState::DSt || tSwing <= 0.0) return; // no swing: hold the foot

    // Cycloid c(phi) = (2*pi*phi - sin(2*pi*phi)) / (2*pi) goes 0 -> 1 with zero
    // velocity at both ends; the z bump 0.5*h*(1 - cos(2*pi*phi)) peaks at phi = 0.5.
    // Derivatives w.r.t. time use dphi/dt = 1 / tSwing.
    const double w = 2.0 * M_PI;
    const double c = (w * phi - std::sin(w * phi)) / w;
    const double dc = 1.0 - std::cos(w * phi);
    const double ddc = w * std::sin(w * phi);
    const double bump = 0.5 * stepHeight * (1.0 - std::cos(w * phi));
    const double dbump = 0.5 * stepHeight * w * std::sin(w * phi);
    const double ddbump = 0.5 * stepHeight * w * w * std::cos(w * phi);
    const double T = tSwing, T2 = tSwing * tSwing;

    const Eigen::Vector3d delta = posDes_W - posStart_W;
    if (!inPlaceOnly) {
        pDes_.head<2>() += delta.head<2>() * c;
        vDes_.head<2>() = delta.head<2>() * dc / T;
        aDes_.head<2>() = delta.head<2>() * ddc / T2;
    }
    // z always lifts, also in place
    pDes_.z() += bump + delta.z() * c;
    vDes_.z() = (dbump + delta.z() * dc) / T;
    aDes_.z() = (ddbump + delta.z() * ddc) / T2;
}

void FootPlacement::planFootstep()
{
    // Step length from the commanded forward speed: the body advances one
    // stepLength per swing, so its speed is stepLength / tSwing. Directions use
    // the commanded heading, not the base yaw (wobbles while the CoM sways) or
    // the foot yaw (the standing pose has the toes slightly out).
    const double c = std::cos(heading_), s = std::sin(heading_);
    const double vForward = c * desV_W.x() + s * desV_W.y();
    stepLength = std::clamp(vForward * tSwing, -maxStepLength, maxStepLength);

    // Land stepLength ahead of the stance foot and stanceWidth beside it;
    // relative to the stance foot, so the feet leapfrog without drifting
    const double side = (legState == LegState::RSt) ? 1.0 : -1.0; // +1: left foot swings
    const Eigen::Vector2d step_L(stepLength, side * stanceWidth);
    posDes_W.x() = posStance_W.x() + c * step_L.x() - s * step_L.y();
    posDes_W.y() = posStance_W.y() + s * step_L.x() + c * step_L.y();
    // Ground height = stance foot height (flat ground); zOff_W < 0 stretches the
    // swing target below the ground so the foot surely makes contact
    posDes_W.z() = posStance_W.z() + zOff_W;
}

void FootPlacement::StepSwingPlanning(const RobotState &state, const pinocchio::Data &data,
                                      const MyGaitScheduler &gait_scheduler, const JoyStickInterpreter &joyStick)
{
    updateFromRobot(state, data, gait_scheduler.legState, gait_scheduler.phi, gait_scheduler.tSwing, joyStick);
    planFootstep();
    computeSwingTrajectory();
}

void FootPlacement::StepSwingPlanning(const RobotState &state, const pinocchio::Data &data,
                                      const JoyStickInterpreter &joyStick, const CP_Planning &cp_planner)
{
    // Swing leg and phase as published by the CP planner (synchronous with the
    // CoM in planWalking(), one cycle behind it in planWarmingUp())
    updateFromRobot(state, data, cp_planner.leg_state_swing_, cp_planner.phi_swing, cp_planner.t_swing, joyStick);
    planFootstep();
    computeSwingTrajectory();
}

void FootPlacement::StepSwingPlanningRaibert(const RobotState &state, const pinocchio::Data &data,
                                             const MyGaitScheduler &gait_scheduler, const JoyStickInterpreter &joyStick)
{
    updateFromRobot(state, data, gait_scheduler.legState, gait_scheduler.phi, gait_scheduler.tSwing, joyStick);
    if (gait_scheduler.motionState == MotionState::WARM_UP) inPlaceOnly = true;
    else if (gait_scheduler.motionState == MotionState::WALK) inPlaceOnly = false;

    // Raibert heuristic (OpenLoong's getSwingPos()): land where the base will
    // be at touchdown given its current velocity, plus feedback on the error
    // to the commanded velocity
    Eigen::Matrix3d KP = Eigen::Matrix3d::Zero(), Rz;
    KP(0, 0) = kp_vx;
    KP(1, 1) = kp_vy;
    Rz << std::cos(yawCur_), -std::sin(yawCur_), 0,
          std::sin(yawCur_),  std::cos(yawCur_), 0,
          0, 0, 1;
    KP = Rz * KP * Rz.transpose();
    posDes_W = hipPos_W - KP * (desV_W - curV_W) + 0.5 * tSwing * curV_W + curV_W * (1 - phi) * tSwing;

    // Yaw-rate correction: where the hip-offset point will be at touchdown
    // given the current and commanded turning rate
    const double thetaF = yawCur_ + theta0_ + omegaZ_W_ * (1 - phi) * tSwing + 0.5 * omegaZ_W_ * tSwing +
                          kp_wz * (omegaZ_W_ - desWz_W);
    posDes_W.x() += 0.5 * stanceWidth * (std::cos(thetaF) - std::cos(yawCur_ + theta0_));
    posDes_W.y() += 0.5 * stanceWidth * (std::sin(thetaF) - std::sin(yawCur_ + theta0_));
    posDes_W.head<2>() += landingOffset_W();
    // Ground height = stance foot height (flat ground); zOff_W < 0 stretches the
    // swing target below the ground so the foot surely makes contact
    posDes_W.z() = posStance_W.z() + zOff_W;

    computeSwingTrajectory();
}
