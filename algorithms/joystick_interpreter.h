/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include "ramp_trajectory.h"
#include "data_type.h"

class JoyStickInterpreter {
public:
    
    double thetaZ{0.0};
    double thetaY{0.0};
    JoyStickInterpreter(double dtIn): dt{dtIn}, vxLGen(dtIn), vyLGen(dtIn), PzLGen(dtIn), wzLGen(dtIn), thetazGen(dtIn), pitchLGen(dtIn){};
    void setVxDesLPara(double vxDesLIn, double timeToReach);
    void setVyDesLPara(double vyDesLIn, double timeToReach);
    void setPzRef (double pzDesIn, double timeToReach); // control the base height
    void setWzDesLPara(double wzDesLIn, double timeToReach);
    void setPitchRef(double pitchDesIn, double timeToReach); // control base pitch
    void setIniPos(double posX, double posY, double thetaZ);
    void setIniPos(double posX, double posY, 
        double posZ, double thetaZ);
    void setMotionState (MotionState motion_state);
    void step();
    double vx_W{0.0}, vy_W{0.0}, vz_W{0.0}; // generated velocity in x and y direction w.r.t world frame
    double px_W{0.0}, py_W{0.0}, pz_W{0.0}; // generated position in x and y direction w.r.t world frame
    double vx_L{0.0}, vy_L{0.0}, wz_L{0.0}; // generated linear velocity in x and y direction, angular velocity in z direction, w.r.t body frame
    void reset();
    MotionState getMotionState () const {return this->motion_state;};

    RampTrajectory vxLGen, vyLGen, PzLGen, wzLGen, thetazGen, pitchLGen;

private:
    double dt;
    MotionState motion_state;
};


