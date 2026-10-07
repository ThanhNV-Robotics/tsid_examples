#pragma once
#include "Eigen/Dense"
#include "data_type.h"
#include "joystick_interpreter.h"
#include "my_gait_scheduler.h"

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
        WalkPlanner (const double dtIn, const double zIn, double wd_hipIn);
        
        double CoM_dynamics (double cxi, double xc); // dx = f(x,u)
        double CP_dynamics (double p, double cxi);
        void computeCoM (double cxi_x, double cxi_y);

        void planWalking (MyGaitScheduler &gait_scheduler, JoyStickInterpreter &joyStick);
        void computeCP (double zmp_x, double zmp_y);

        // Seed the planner from the robot's actual CoM (and CoM velocity, zero if
        // the robot is standing still)
        void setInitCom (Vector3d com_pos, Vector3d com_vel = Vector3d::Zero());
        Vector3d getCoMref();
        // Planned CoM velocity and acceleration (LIPM: ddc = w^2 (c - p), z = 0),
        // feedforward for the TSID CoM task
        Vector3d getCoMvelRef() const;
        Vector3d getCoMaccRef() const;

    // private:
        double dt_; // sampling time
        double xc_, yc_, zc_; // CoM position
        double d_xc_, d_yc_; // CoM velocity

        double cxi_x_, cxi_y_, cxi_xd_, cxi_yd_; // capture point
        double cxi_x0_, cxi_y0_;

        double zmp_x_d_, zmp_y_d_; // desired zmp 
        LegState leg_state_;

};