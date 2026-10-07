#pragma once
#include "Eigen/Dense"
#include "cheat_state_estimator.h"
#include "data_type.h"
#include "joystick_interpreter.h"

#include <algorithm>
#include <string>
#include <tsid/robots/robot-wrapper.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

class WalkPlanner
{
    public:
        const double g = 9.81; //gravity constant
        double wd_hip = 0.1; // hip width
        double t_swing{0.0};
        double w{0.0};
        double step_length{0.0}; // only set by planWalking(); stays 0 during warm-up
        double max_step_length{0.1}; // |step_length| limit, keep equal to FootPlacement::maxStepLength
        double phi_CoM{0.0}; // CoM phase variable
        double phi_Lfeet{0.0}; // left feet phase variable
        double phi_Rfeet{0.0}; // right feet phase variable

        LegState leg_state_swing_{LegState::DSt};

        double xBias{0.0}, yBias{0.0};
        // Parameters from the "walk_planner: foot_placement:" block of config_path;
        // zIn = initial CoM height above the ground
        WalkPlanner (const std::string &config_path, const double zIn);
        
        double CoM_dynamics (double cxi, double xc); // dx = f(x,u)
        double CP_dynamics (double p, double cxi);
        void computeCoM (double cxi_x, double cxi_y);

        void planWalking (JoyStickInterpreter &joystick, Cheat_StateEstimator &state_estimator,
            tsid::robots::RobotWrapper &robot_wrapper, const pinocchio::Data &data, double Tswing);

        void computeCP (double zmp_x, double zmp_y);

        // Seed the planner from the robot's actual CoM (and CoM velocity, zero if
        // the robot is standing still)
        void setInitCom (Vector3d com_pos, Vector3d com_vel = Vector3d::Zero());
        Vector3d getCoMref();
        // Planned CoM velocity and acceleration (LIPM: ddc = w^2 (c - p), z = 0),
        // feedforward for the TSID CoM task
        Vector3d getCoMvelRef() const;
        Vector3d getCoMaccRef() const;
        Vector3d getZMPref () const;
        Vector3d getCapturedPointref () const;
        Vector3d getPhase () const; // return phiCom, phi_Lfeet, phi_Rfeet

        // ---- Outputs in the world frame (the getters above are in the current
        // planning frame: pelvis while shifting, stance ankle while stepping) ----
        Vector3d getCoMrefW () const;      // z = planned CoM height above the floor
        Vector3d getCoMvelRefW () const;
        Vector3d getCoMaccRefW () const;
        Vector3d getZMPrefW () const;      // z = 0 (floor)
        Vector3d getCapturedPointrefW () const;
        Vector3d getNextFootW () const;    // swing ankle landing point, z = stance ankle height

        // ---- Gait state for the swing planner and the TSID contact switch ----
        // Stance leg of the current step: LSt = left stands / right swings,
        // RSt = right stands / left swings, DSt = not stepping yet (init shift)
        LegState getStanceLeg () const { return stance_leg_; }
        // Phase of the current step in [0, 1] (0 while not stepping)
        double getSwingPhase () const { return stance_leg_ == LegState::DSt ? 0.0 : std::min(phi_CoM, 1.0); }
        // Step event, true only on the tick a new step starts: the previous swing
        // foot has touched down (if there was one) and the other foot lifts off
        bool stanceChanged () const { return stance_changed_; }

    private:
        WalkPlanner (const YAML::Node &cfg, const double zIn);

        // Re-express CoM, CP and ZMP (position and velocity, xy only) in a new
        // planning frame: origin o_new_W (world xy), x axis at yaw_new
        void reanchor (const Eigen::Vector2d &o_new_W, double yaw_new);

        // Planning frame -> world: points (origin + rotation) and directions (rotation only)
        Eigen::Vector2d pointToWorld (double x, double y) const;
        Eigen::Vector2d dirToWorld (double x, double y) const;

        // Start a single-support step on `stance` (LSt / RSt): re-anchor the
        // planning frame to that ankle, capture xi0, reset the phase
        void enterStance (LegState stance, const tsid::robots::RobotWrapper &robot_wrapper,
                          const pinocchio::Data &data);

        bool isInit_{false};
        bool initStepping {false};
        double ref_theta; // current heading angle to remain
        double dt_; // sampling time
        double xc_, yc_, zc_; // CoM position
        double d_xc_, d_yc_; // CoM velocity

        double cxi_x_, cxi_y_, cxi_xd_, cxi_yd_; // capture point
        double cxi_x0_, cxi_y0_;

        double zmp_x_d_, zmp_y_d_; // desired zmp
        // Current planning frame (frozen per phase): origin in world xy and yaw
        Eigen::Vector2d frame_origin_W_{Eigen::Vector2d::Zero()};
        double frame_yaw_{0.0};
        LegState stance_leg_{LegState::DSt}; // leg the planning frame is anchored to (DSt = pelvis)
        bool stance_changed_{false}; // step event of this tick, see stanceChanged()
        double stance_z_W_{0.0};     // stance ankle height (world z), captured on entering stance
        bool swing_lifted_{false};   // swing foot has left the ground during this step
        double phi_td_min_{0.7};     // earliest step phase at which a touchdown is accepted

        // ZMP limits in the stance-ankle frame: sole box of the ankle link's URDF
        // collision (x in [-0.05, 0.13], |y| <= 0.03) shrunk by a 1 cm margin
        double sole_x_min_{-0.04}, sole_x_max_{0.12}, sole_y_half_{0.02};

        Eigen::Vector2d next_foot_pos_{Eigen::Vector2d::Zero()}; // next swing foothold, w.r.t. the stance ankle (stance-foot axes)
        const std::string left_foot_frame;
        const std::string right_foot_frame;


};