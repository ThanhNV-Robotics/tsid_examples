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
MyGaitScheduler::MyGaitScheduler(const double Tswing, double dt)
{
    this->tSwing = Tswing;
    this->dt = dt;
    phi = 0;
    isIni = false;
	firstleg=LegState::LSt;
    legState=LegState::DSt;
    legStateNext=firstleg;
    motionState=MotionState::STAND;
    enableNextStep= false;
    touchDown = false;
}

void MyGaitScheduler::step()
{
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

















