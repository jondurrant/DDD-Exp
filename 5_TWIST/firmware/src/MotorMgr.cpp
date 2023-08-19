/*
 * MotorMgr.cpp
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#include "MotorMgr.h"
#include <math.h>

MotorMgr::MotorMgr(uint8_t gpCW, uint8_t gpCCW,
		uint8_t gpA, uint8_t gpB) {
	xGPCW= gpCW;
	xGPCCW = gpCCW;
	xGPA = gpA;
	xGPB = gpB;

	gpio_init(xGPCW);
	gpio_set_function(xGPCW, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPCW, 0);
	uint slice_num = pwm_gpio_to_slice_num(xGPCW);
	pwm_set_enabled(slice_num, true);

	gpio_init(xGPCCW);
	gpio_set_function(xGPCCW, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPCCW, 0);
	slice_num = pwm_gpio_to_slice_num(xGPCCW);
	pwm_set_enabled(slice_num, true);

	GPIOInputMgr::getMgr()->addObserver(xGPA, this);
	GPIOInputMgr::getMgr()->addObserver(xGPB, this);

}

MotorMgr::~MotorMgr() {
	// TODO Auto-generated destructor stub
}


void MotorMgr::setThrottle(float percent, bool cw){
	xThrottle = percent;
	xCW = cw;

	if (xThrottle < 0 ){
		xThrottle == 0.0;
	}

	if (xThrottle == 0.0){
		xActRPM = 0.0;
		xLastTime = 0;
		pwm_set_gpio_level(xGPCW, 0);
		pwm_set_gpio_level(xGPCCW, 0);
		//printf("PWM STOP\n");
		return;
	}

	if (xThrottle > 1.0 ){
		xThrottle = 1.0;
	}

	int pwm = (int)((float)(0xffff) * xThrottle);
	if (cw){
		pwm_set_gpio_level(xGPCCW, 0);
		pwm_set_gpio_level(xGPCW, pwm);
		//printf("GP%d, pwm %d\n", xGPCW, pwm);
	} else {
		pwm_set_gpio_level(xGPCW, 0);
		pwm_set_gpio_level(xGPCCW, pwm);
		//printf("GP%d, pwm %d\n", xGPCCW, pwm);
	}
	//printf("PWM %d, Throttle %0.2f\n", pwm, xThrottle);
}

void MotorMgr::handleGPIO(uint gpio, uint32_t events){
	if (gpio == xGPSlot){
		handleCalibration(gpio, events);
		return;
	}


	uint8_t c;
	c = gpio_get(xGPA);
	c = c << 1;
	c = (gpio_get(xGPB)) | c;

	if (xRotEncCW[xLast] == c){
		xCount++;
		if (xCount > 3){
			xPos++;
			xDeltaPos++;
			if (xPos == xNumTicks){
				xPos = 0;
			}
			//printf("Clockwise %d %d\n", xPos, xCount);
			handleRotate(true);
			xCount = 0;
		}

		xLast = c;
	}

	if (xRotEncCCW[xLast] == c){
		xCount-- ;
		if (xCount < -3){
			xPos--;
			xDeltaPos--;
			if (xPos == -1){
				xPos = xNumTicks - 1;
			}
			//printf("Withershins %d %d\n", xPos, xCount);
			handleRotate(false);
			xCount = 0;
		}

		xLast = c;
	}
}

void MotorMgr::handleRotate(bool cw){
	uint32_t now = to_ms_since_boot (get_absolute_time ());

	if (xLastTime != 0){
		uint32_t ms = now - xLastTime;
		float rpm = 60000.0 / (float)ms;
		rpm = rpm / (float)xNumTicks;
		xActRPM = rpm;
		xMvAvgRPM = (rpm * 1.0 + xMvAvgRPM * 3.0)/ 4.0;

	}
	xLastTime = now;
}

void MotorMgr::calibrate(uint8_t gpSlot){
	xGPSlot = gpSlot;
	xNumTicks = 0xFFFF;
	GPIOInputMgr::getMgr()->addObserver(xGPSlot, this);

}

void MotorMgr::handleCalibration(uint gpio, uint32_t events){
	if (xPos == 0){
		return;
	}
	if ((events & GPIO_IRQ_EDGE_RISE) > 0){
		uint32_t now = to_ms_since_boot (get_absolute_time ());
		uint32_t ms = now - xSlotTime;
		float rpm = 60000.0 / (float)ms;
		xSlotTime = now;

		//printf("TICK COUNT %d RPM %0.2f, Reported RPM %0.2f Error %0.2f\n",
		//		xPos, rpm, xActRPM, rpm - xActRPM);
		xPos = 0;
	}
}


float MotorMgr::getThrottle(){
	return xThrottle;
}


float MotorMgr::getRPM(){


	//Check we have recently updated RPM, otherwise we are stopped
	uint32_t now = to_ms_since_boot (get_absolute_time ());
	uint32_t ms = now - xLastTime;
	if (ms > 250){
		xActRPM = 0.0;
	}



	return xActRPM;

}
bool MotorMgr::isCW(){
	return xCW;
}


float MotorMgr::getMovingAvgRPM(){
	if (getRPM() == 0.0){
		xMvAvgRPM = 0.0;
	}
	return xMvAvgRPM;
}


/***
 * Radian possition of wheel
 * @return 0.0 >= r < 2* PI
 */
float MotorMgr::getRadians(){
	float rad = (float)xPos / (float)xNumTicks;
	rad = rad * (2.0 * M_PI);
	return rad;
}


float MotorMgr::getAvgRadPerSec(){
	float rpm = getMovingAvgRPM();
	float rps = (rpm / 60.0) * (2.0 * M_PI);

	return rps;
}


/***
 * Get the delta of ticks since last cleared
 * @param clear - if true will be reset to zero after call
 * @return number of ROTENC ticks
 */
int32_t MotorMgr::getDeltaPos(bool clear){
	int32_t res = xDeltaPos;
	if (clear){
		xDeltaPos = 0;
	}
	return res;
}

/***
 * Get the delta of radians since last cleared
 * @param clear - if true will be reset to zero after call
 * @return Radians turn since last call (>0 CW, <0 CCW).
 */
float MotorMgr::getDeltaRadians(bool clear){
	float res = (float)getDeltaPos(clear);
	res = res / (float)xNumTicks;
	res = res * (2.0 * M_PI);
	return res;
}


