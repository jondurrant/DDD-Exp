/*
 * MotorPID.h
 *
 *  Created on: 29 May 2023
 *      Author: jondurrant
 */

#ifndef ENCODER_SRC_MOTORPID_H_
#define ENCODER_SRC_MOTORPID_H_

#include "MotorMgr.h"

class MotorPID : public MotorMgr {
public:
	/***
	 * Constructor
	 * @param gpCW - GP Pad for PWM for Clockwise
	 * @param gpCCW - GP Pad for PWM for Clockwise
	 * @param gpA - GP Pad for ROTENC A
	 * @param gpB - GP Pad for ROTENC AB
	 */
	MotorPID(uint8_t gpCW, uint8_t gpCCW,
			uint8_t gpA, uint8_t gpB);
	virtual ~MotorPID();

	/***
	 * Set motor speed in Revs per min
	 * @param rpm - Revolutions per minute
	 * @param cw - True for clockwise
	 */
	void setSpeedRPM(float rpm, bool cw);


	/***
	 * Set motor speed in Radians per second
	 * @param rps - Radians Per Second
	 * @param cw - True for clockwise
	 */
	void setSpeedRadPS(float rps, bool cw);

	/***
	 * Set the constants for the PID calculation
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	void configPID(float kP, float kI, float kD);

	/***
	 * Use PID to update the throttle
	 * @return current error level between set point and process value
	 */
    float doPID();

	/***
	 * Do the PIC calculate and return the component parts.
	 * @param sp
	 * @param pv
	 * @param err
	 * @param p
	 * @param i
	 * @param d
	 * @return
	 */
	float pid (float &sp, float &pv, float &err,
			float &p, float &i, float &d);

protected:

	virtual void handleRotate(bool cw);



	float xTargetRPM = 0.0;



private:

	float xPrevError;
	float xKP = 1;
	float xKI = 1;
	float xKD = 1;
	float xCumErr = 0.0;
	float xLastErr = 0.0;

};

#endif /* ENCODER_SRC_MOTORPID_H_ */
