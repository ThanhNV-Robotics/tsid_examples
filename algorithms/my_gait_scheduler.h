#pragma once

#include <Eigen/Dense>
#include <string>
#include "useful_math.h"

#include "data_type.h"
#include "robot_wrapper.h"
#include "joystick_interpreter.h"

class MyGaitScheduler {
public:
    bool isIni{false};
	bool start_walk{true};
    double phi{0};
    double tSwing{0.8};
    double dt{0.001};
    double FzThrehold{100};
    double Fz_L_m{0}, Fz_R_m{0};
    // DataBus::LegState firstleg, legState, legStateNext;
    LegState firstleg, legState, legStateNext;
    MotionState motionState;
    // yamlPath's "gait_scheduler:" block supplies tSwing -- see
    // config/step_planning_cf.yaml
    MyGaitScheduler(const std::string &yamlPath, double dtIn);
    void step(JoyStickInterpreter &joyStick);
    void stop();
	void start(JoyStickInterpreter &joystick);
    Eigen::VectorXd FLest,FRest;
    Eigen::VectorXd torJoint;
    bool enableNextStep;
    bool touchDown; // touch down event indicator
    int stepNumDes{1}, stepNumCur{0};

private:
    Eigen::VectorXd fe_r_pos_W, fe_l_pos_W, swingStartPos_W, posHip_W, posST_W, hip_r_pos_W, hip_l_pos_W, dq;
    Eigen::VectorXd stanceStartPos_W;
    Eigen::MatrixXd fe_r_rot_W, fe_l_rot_W;
    Eigen::MatrixXd dyn_M, dyn_Non, J_l, J_r, dJ_l, dJ_r;
    double theta0;
    int model_nv;
};