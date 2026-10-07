/*
Adapted from OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
*/

#pragma once

#include <Eigen/Dense>
#include <string>
#include <tsid/robots/robot-wrapper.hpp>

#include "data_type.h"
#include "joystick_interpreter.h"
#include "walk_planner.h"

// Swing-foot planner: picks the landing position of the swing foot and
// generates a cycloid swing trajectory (position, velocity, acceleration in
// the world frame) for the swing foot's contact frame, ready to be used as
// the reference of a TSID TaskSE3Equality.
//
// Leg-state convention (same as the gait scheduler / CP planner):
//   LSt -> left foot is stance, RIGHT foot swings
//   RSt -> right foot is stance, LEFT foot swings
//   DSt -> double support, no foot swings (both reference outputs hold still)
//
// Call one StepSwingPlanning() overload per control step, after
// tsid.computeProblemData() so that `data` holds the current kinematics.
class FootPlacement
{
public:
    // ---- Parameters (optional "foot_placement:" block of the YAML) ----
    double stepHeight{0.05};   // apex height of the swing foot above its liftoff height [m]
    double stanceWidth{0.27};  // nominal lateral distance between the feet [m]
    double xOff_L{0.0};        // body-frame forward landing offset [m]
    double yOff_L{0.0};        // body-frame lateral landing offset, positive = inward [m]
    double zOff_W{0.0};        // touchdown stretch: landing target relative to the ground (stance foot height) [m]; < 0 reaches below the ground
    double kp_vx{0.0}, kp_vy{0.0}, kp_wz{0.0}; // Raibert velocity-feedback gains
    double maxStepLength{0.1}; // |stepLength| limit [m]
    std::string leftFootFrame{"left_ankle_pitch_link"};
    std::string rightFootFrame{"right_ankle_pitch_link"};

    // When true, x/y stay at the liftoff position: the foot only lifts and
    // lowers in place. Not needed for stepping in place: a zero velocity
    // command already gives a zero step length.
    bool inPlaceOnly{false};

    // When true, every swing starts from the previous PLANNED footsteps
    // instead of the measured feet: the planner walks open loop, so the
    // footstep sequence can be checked while the real feet stay planted.
    // Initialised from the measured feet at the first call.
    bool openLoopFootsteps{false};

    // ---- State of the current swing (world frame) ----
    LegState legState{LegState::DSt};
    double phi{0};       // swing phase in [0, 1]
    double tSwing{0.4};  // swing duration [s]
    Eigen::Vector3d posStart_W{Eigen::Vector3d::Zero()}; // swing foot position at liftoff
    Eigen::Vector3d posStance_W{Eigen::Vector3d::Zero()}; // stance foot position at liftoff
    double stepLength{0};  // forward distance from the stance foot to the landing point [m]
    Eigen::Vector3d posDes_W{Eigen::Vector3d::Zero()};   // planned landing position
    Eigen::Vector3d hipPos_W{Eigen::Vector3d::Zero()};   // nominal foot position under the swing-side hip
    Eigen::Vector3d base_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d curV_W{Eigen::Vector3d::Zero()};     // measured base velocity
    Eigen::Vector3d desV_W{Eigen::Vector3d::Zero()};     // commanded base velocity
    double desWz_W{0};

    FootPlacement(const std::string &yamlPath, const tsid::robots::RobotWrapper &robot);

    // Swing leg, phase and landing point from the walk planner (stance leg,
    // step phase, next foothold); the trajectory is the same cycloid
    void StepSwingPlanning(const RobotState &state, const pinocchio::Data &data,
                           const JoyStickInterpreter &joyStick, const WalkPlanner &walk_planner);

    // ---- Swing-foot reference (world frame) ----
    const Eigen::Vector3d &getSwingDesPos() const { return pDes_; }
    const Eigen::Vector3d &getSwingDesVel() const { return vDes_; }
    const Eigen::Vector3d &getSwingDesAcc() const { return aDes_; }

    // Which foot is swinging: true for the left foot (legState == RSt)
    bool isLeftSwing() const { return legState == LegState::RSt; }
    bool isSwinging() const { return legState != LegState::DSt; }
    const std::string &getSwingFrameName() const { return isLeftSwing() ? leftFootFrame : rightFootFrame; }

private:
    const tsid::robots::RobotWrapper &robot_;
    pinocchio::FrameIndex leftFootId_, rightFootId_;

    double yawCur_{0}, omegaZ_W_{0}, theta0_{0};
    double heading_{0}; // commanded walking heading (joystick thetaZ)
    LegState swingLegPrev_{LegState::DSt};
    Eigen::Vector3d plannedLeft_W{Eigen::Vector3d::Zero()}, plannedRight_W{Eigen::Vector3d::Zero()};
    bool swingInit_{false};

    Eigen::Vector3d pDes_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vDes_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d aDes_{Eigen::Vector3d::Zero()};

    // Reads the robot state, snapshots the liftoff position when the swing leg
    // changes, and computes the nominal foot position hipPos_W
    void updateFromRobot(const RobotState &state, const pinocchio::Data &data, LegState curLegState,
                         double phiIn, double tSwingIn, const JoyStickInterpreter &joyStick);
    // Cycloid from posStart_W to posDes_W with a stepHeight bump in z
    void computeSwingTrajectory();
};
