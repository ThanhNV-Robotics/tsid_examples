/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "my_gait_scheduler.h"
#include "data_type.h"
#include "joystick_interpreter.h"
#include <yaml-cpp/yaml.h>

// Constructor
// Note: no double-support here, swing time always equals to stance time
MyGaitScheduler::MyGaitScheduler(const std::string &yamlPath, double dtIn)
{
    YAML::Node root = YAML::LoadFile(yamlPath);
    tSwing = root["gait_scheduler"]["tSwing"].as<double>();
    dt = dtIn;
    phi = 0;
    isIni = false;
	firstleg=LegState::LSt;
    legState=LegState::DSt;
    legStateNext=firstleg;
    motionState=MotionState::STAND;
    enableNextStep= false;
    touchDown = false;
}

// void MyGaitScheduler::dataBusRead(const DataBus &robotState)
// {
// 	if (motionState != DataBus::Stand && stepNumCur == 0)
// 		legState=firstleg;
//     model_nv = robotState.model_nv;
//     torJoint = Eigen::VectorXd::Zero(model_nv - 6);
//     for (int i = 0; i < model_nv - 6; i++)
//     {
//         torJoint[i] = robotState.motors_tor_cur[i];
//     }
//     dyn_M = robotState.dyn_M;
//     dyn_Non = robotState.dyn_Non;
//     J_l = robotState.J_l;
//     dJ_l = robotState.dJ_l;
//     J_r = robotState.J_r;
//     dJ_r = robotState.dJ_r;
//     Fz_L_m = robotState.fL[2];
//     Fz_R_m = robotState.fR[2];
//     hip_l_pos_W = robotState.hip_l_pos_W;
//     hip_r_pos_W = robotState.hip_r_pos_W;
//     fe_r_pos_W = robotState.fe_r_pos_W;
//     fe_l_pos_W = robotState.fe_l_pos_W;
//     fe_l_rot_W = robotState.fe_l_rot_W;
//     fe_r_rot_W = robotState.fe_r_rot_W;
//     dq = robotState.dq;
//     motionState = robotState.motionState;
// }

// void MyGaitScheduler::dataBusWrite(DataBus &robotState)
// {
//     robotState.tSwing = tSwing;
//     robotState.swingStartPos_W = swingStartPos_W;
//     robotState.stanceDesPos_W = stanceStartPos_W;
//     robotState.posHip_W = posHip_W;
//     robotState.posST_W = posST_W;
//     robotState.theta0 = theta0;
//     robotState.legState = legState;
//     robotState.legStateNext = legStateNext;
//     robotState.phi = phi;
//     robotState.FL_est = FLest;
//     robotState.FR_est = FRest;
//     if (legState == DataBus::LSt)
//     {
//         robotState.stance_fe_pos_cur_W = fe_l_pos_W;
//         robotState.stance_fe_rot_cur_W = fe_l_rot_W;
//     }
//     else if (legState == DataBus::RSt){
//         robotState.stance_fe_pos_cur_W=fe_r_pos_W;
//         robotState.stance_fe_rot_cur_W=fe_r_rot_W;
//     }
//     robotState.motionState = motionState;
// }

void MyGaitScheduler::step(JoyStickInterpreter &joyStick)
{
    // Eigen::VectorXd tauAll;
    // tauAll = Eigen::VectorXd::Zero(model_nv);
    // tauAll.block(6, 0, model_nv - 6, 1) = torJoint; //
    // FLest = -pseudoInv_SVD(J_l * dyn_M.inverse() * J_l.transpose()) * (J_l * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_l * dq);
    // FRest = -pseudoInv_SVD(J_r * dyn_M.inverse() * J_r.transpose()) * (J_r * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_r * dq);

    double dPhi{0};

    if (motionState == MotionState::WALK_TO_STAND)
    {
        enableNextStep = false;
		start_walk = false;
        if (touchDown)
            motionState = MotionState::STAND;
    }

    if (motionState == MotionState::STAND)
    {
        dPhi = 0;
        phi = 0; // need to refined
        isIni = false;
        enableNextStep = false;
        stepNumCur=0;
    }
    else if (motionState == MotionState::WALK)
    {
        enableNextStep = true;
        dPhi = 1.0 / tSwing * dt;
    }
    else if (motionState == WARM_UP) // prepare to walk, swing the com in y direction
    {
        enableNextStep = true;
        dPhi = 1.0 / tSwing * dt;
    }
    else if (motionState == MotionState::WALK_TO_STAND)
        dPhi = 1.0 / tSwing * dt;

    phi += dPhi;
    if (enableNextStep)
        touchDown = false;

    if (!isIni &&  start_walk)
    {
        isIni = true;
		legState = firstleg;
        if (legState == LegState::LSt)
        { // here define which leg support first
            swingStartPos_W = fe_r_pos_W;
            stanceStartPos_W = fe_l_pos_W;
        }
        else
        {
            swingStartPos_W = fe_l_pos_W;
            stanceStartPos_W = fe_r_pos_W;
        }
    }

    // if (legState == DataBus::LSt && FRest[2] >= 280 && phi >= 0.6)
    // if (legState == DataBus::LSt && ((FRest[2] >= 280 && phi >= 0.6) || (phi >=0.99)))
    // if (legState == DataBus::LSt && phi >= 0.9)

    if (legState == LegState::LSt && phi >= 1.0)
    {
        if (enableNextStep)
        {
            // std::cout << "#######right" << std::endl;
            legState = LegState::RSt;
            swingStartPos_W = fe_l_pos_W;
            stanceStartPos_W = fe_r_pos_W;
            phi = 0;
            stepNumCur++;
        }
    }
    // else if (legState == DataBus::RSt && FLest[2] >= 280 && phi >= 0.6)
    // else if (legState == DataBus::RSt && ((FLest[2] >= 280 && phi >= 0.6) || (phi >=0.99)))
    // else if (legState == DataBus::RSt && phi >= 0.9)
    else if (legState == LegState::RSt && phi >= 1.0)
    {
        if (enableNextStep)
        {
            // std::cout << "#######left" << std::endl;
            legState = LegState::LSt;
            swingStartPos_W = fe_r_pos_W;
            stanceStartPos_W = fe_l_pos_W;
            phi = 0;
			stepNumCur++;
        }
    }

    if (!enableNextStep)
    {
        // if (legState == DataBus::LSt && FRest[2] >= 200)
        if (legState == LegState::LSt && phi >= 1.0)
        {
            touchDown = true;
            stepNumCur++;
			legState = LegState::DSt;
        }
        // if (legState == DataBus::RSt && FLest[2] >= 200)
        if (legState == LegState::RSt && phi >= 1.0)
        {
            touchDown = true;
            stepNumCur++;
			legState = LegState::DSt;
        }
    }

    if (phi >= 1)
    {
        phi = 1;
    }
    if (legState == LegState::LSt)
    {
        posHip_W = hip_r_pos_W;
        posST_W = fe_l_pos_W;
        theta0 = -3.1415 * 0.5;
        legStateNext = LegState::RSt;
		if (motionState == MotionState::WALK)
        	legStateNext = LegState::RSt;
		else if (motionState == MotionState::WALK_TO_STAND)
			legStateNext = LegState::DSt;
    }
    else if (legState == LegState::RSt)
    {
        posHip_W = hip_l_pos_W;
        posST_W = fe_r_pos_W;
        theta0 = 3.1415 * 0.5;
        legStateNext = LegState::LSt;
		if (motionState==MotionState::WALK)
        	legStateNext = LegState::LSt;
		else if (motionState == MotionState::WALK_TO_STAND)
			legStateNext = LegState::DSt;
    }
	else{
		posHip_W = hip_l_pos_W;
		posST_W=fe_r_pos_W;
		theta0=3.1415*0.5;
		legStateNext = LegState::DSt;
	}

}

void MyGaitScheduler::start(JoyStickInterpreter &joystick){
	start_walk = true;
    this->motionState = joystick.getMotionState();
}

















