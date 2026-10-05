#pragma once
#include "Eigen/Dense"
#include "data_type.h"
#include "joystick_interpreter.h"
#include "my_gait_scheduler.h"

class CP_Planning
{
    public:
        const double g = 9.81; //gravity constant
        double wd_hip = 0.1; // hip width
        double t_swing;
        double w;
        double step_length;
        double phi_swing{0.0};
        LegState leg_state_swing_{LegState::DSt};
        double swayAmplitudeScale{1.0};
        double xBias{0.0}, yBias{0.0};
        CP_Planning (const double dtIn, const double zIn, double wd_hipIn);
        
        double CoM_dynamics (double cxi, double xc); // dx = f(x,u)
        double CP_dynamics (double p, double cxi);
        void computeCoM (double cxi_x, double cxi_y);
        void planWarmingUp (MyGaitScheduler &gait_scheduler);
        void planWalking (MyGaitScheduler &gait_scheduler, JoyStickInterpreter &joyStick);
        void computeCP (double zmp_x, double zmp_y);
        void planSwaySin (MyGaitScheduler &gait_scheduler, double centerY);
        void setInitCom (Vector3d com_pos);
        double swayAmplitude{0.05};     // meters, sine amplitude around centerY
        double swayCyclesPerPhase{1.0}; // number of full left-right-left sine cycles per gait phase (phi: 0->1)
        Vector3d getCoMref();

    // private:
        double dt_; // sampling time
        double xc_, yc_, zc_; // CoM position
        double d_xc_, d_yc_; // CoM velocity

        double cxi_x_, cxi_y_, cxi_xd_, cxi_yd_; // capture point
        double cxi_x0_, cxi_y0_;

        double px_d_, py_d_; // desired zmp 
        LegState leg_state_;

};