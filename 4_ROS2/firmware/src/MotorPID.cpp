/*
 * MotorPID.cpp
 *
 *  Created on: 29 May 2023
 *      Author: jondurrant
 */

#include "MotorPID.h"
#include <cmath>

MotorPID::MotorPID(uint8_t gpCW, uint8_t gpCCW,
		uint8_t gpA, uint8_t gpB) :
		MotorMgr(gpCW, gpCCW, gpA, gpB){
	// NOP

}

MotorPID::~MotorPID() {
	// TODO Auto-generated destructor stub
}

void MotorPID::setSpeedRPM(float rpm, bool cw){
	xTargetRPM = rpm;
	xCW = cw;
	xCumErr = 0.0;
	xLastErr = 0.0;
	//doPID();
}

void MotorPID::setSpeedRadPS(float rps, bool cw){
	float rpm = rps / (M_PI *2.0);
	rpm = rpm * 60.0;
	setSpeedRPM(rpm, cw);
}

void MotorPID::handleRotate(bool cw){
	MotorMgr::handleRotate(cw);
	//doPID();
}

void MotorPID::configPID(float kP, float kI, float kD){
	xKP = kP;
	xKI = kI;
	xKD = kD;
}


float MotorPID::doPID(){

	// Throttle 1.0 = 320RPM
	// Throttle 0.2 = 40RPM
	float error ;
	float sp, pv;
	float p, i, d;

	float pid = this->pid(sp, pv, error, p, i, d);
	xCumErr += error;
	xLastErr = error;


	//printf("PID err: %0.2f p: %0.2f\n", error, p);


	float delt = pid / 320.0;
	float t = getThrottle() + delt;
	//printf("d=%0.2f t=%0.2f\n", delt, t);
	if (t < 0.0){
		t = 0.0;
	}

	setThrottle(t, xCW);
	return t;
}

float MotorPID::pid (float &sp, float &pv, float &err,
		float &p, float &i, float &d){
	sp = xTargetRPM;
	pv = getMovingAvgRPM(); //getRPM();
	err = sp - pv;
	p = err * xKP;

	//xCumErr += err;

	i = xKI * (xCumErr + err);


	d = xKD * (err - xLastErr);

	return p + i + d;

}
