/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "joystick_interpreter.h"

void JoyStickInterpreter::setVxDesLPara(double vxDesLIn, double timeToReach) {
    vxLGen.setPara(vxDesLIn, timeToReach);
}

void JoyStickInterpreter::setVyDesLPara(double vyDesLIn, double timeToReach) {
    vyLGen.setPara(vyDesLIn, timeToReach);
}

void JoyStickInterpreter::setWzDesLPara(double wzDesLIn, double timeToReach) {
    wzLGen.setPara(wzDesLIn, timeToReach);
}

void JoyStickInterpreter::setPzRef (double pzDesIn, double timeToReach) // control the base height
{
    PzLGen.setPara(pzDesIn, timeToReach);
}

void JoyStickInterpreter::setPitchRef(double pitchDesIn, double timeToReach) // control base pitch
{
    pitchLGen.setPara(pitchDesIn, timeToReach);
}


void JoyStickInterpreter::step() {
    vx_L=vxLGen.step();
    vy_L=vyLGen.step();
    pz_W = PzLGen.step();
    thetaY = pitchLGen.step();

    wz_L=wzLGen.step();


    thetaZ=thetaZ+wz_L*dt;
    vx_W=cos(thetaZ)*vx_L-sin(thetaZ)*vy_L;
    vy_W=sin(thetaZ)*vx_L+cos(thetaZ)*vy_L;
    px_W+=vx_W*dt;
    py_W+=vy_W*dt;
}
void JoyStickInterpreter::setMotionState (MotionState motion_state)
{
    this->motion_state = motion_state;
}

void JoyStickInterpreter::reset() {
    vxLGen.resetOut(0);
    vyLGen.resetOut(0);
    wzLGen.resetOut(0);
    pitchLGen.resetOut(0);
    vx_L=0;
    vy_L=0;
    wz_L=0;
    thetaZ=0;
    thetaY=0;
}

void JoyStickInterpreter::setIniPos(double posX, double posY, double thetaZ) {
    px_W=posX;
    py_W=posY;
    this->thetaZ=thetaZ;
}

void JoyStickInterpreter::setIniPos(
    double posX,
    double posY,
    double posZ,
    double thetaZ) {
    px_W = posX;
    py_W = posY;
    pz_W = posZ;
    this->thetaZ = thetaZ;

    // pz_W is overwritten every step() by PzLGen.step() (a RampTrajectory),
    // whose own internal starting point (yOld) is otherwise left at its
    // constructor default of 0 -- without this, the very next step() call
    // discards the posZ just set above and the height ramps from 0 toward
    // whatever setPzRef()'s target is, instead of from posZ.
    PzLGen.resetOut(posZ);
}





