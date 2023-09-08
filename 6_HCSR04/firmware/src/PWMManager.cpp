/*
 * PWMManager.cpp
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#include "PWMManager.h"
#include "stdio.h"
#include "hardware/pwm.h"

PWMManager::PWMManager(uint8_t gpCW, uint8_t gpCCW) {
	xGPCW= gpCW;
	xGPCCW = gpCCW;

	xSwitches.addSwitch(GP_UP);
	xSwitches.addSwitch(GP_DOWN);
	xSwitches.addSwitch(GP_STOP);
	xSwitches.setObserver(this);

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

}

PWMManager::~PWMManager() {
	// TODO Auto-generated destructor stub
}

/***
 * Handle a short press from the switch
 * @param gp - GPIO number of the switch
 */
void PWMManager::handleShortPress(uint8_t gp){
	handlePress(gp);
}

/***
 * Handle a short press from the switch
 * @param gp - GPIO number of the switch
 */
void PWMManager::handleLongPress(uint8_t gp){
	handlePress(gp);
}

/***
 * Handle a  press from the switch
 * @param gp - GPIO number of the switch
 */
void PWMManager::handlePress(uint8_t gp){
	if (gp == GP_STOP){
		xRPM = 0.0;
		setSpeed();
	}
	if (gp == GP_UP){
		xRPM += RPM_INC;
		setSpeed();
	}
	if (gp == GP_DOWN){
		xRPM -= RPM_INC;
		setSpeed();
	}
}


void PWMManager::setSpeed(){
	if (xRPM == 0.0){
		pwm_set_gpio_level(xGPCW, 0);
		pwm_set_gpio_level(xGPCCW, 0);
		printf("PWM STOP\n");
		return;
	}
	if (xRPM > 0 ){
		if (xRPM < SPEED_MIN){
			xRPM = SPEED_MIN;
		}
		if (xRPM > SPEED_MAX) {
			xRPM = SPEED_MAX;
		}
	}

	if (xRPM < 0 ){
		if (xRPM > SPEED_MIN * -1.0){
			xRPM = SPEED_MIN * -1.0;
		}
		if (xRPM < SPEED_MAX * -1.0) {
			xRPM = SPEED_MAX * -1.0;
		}
	}

	int pwm = (int)(xRPM * 50.0);
	pwm = pwm * pwm;
	if (xRPM > 0.0){
		pwm_set_gpio_level(xGPCCW, 0);
		pwm_set_gpio_level(xGPCW, pwm);
	} else {
		pwm_set_gpio_level(xGPCW, 0);
		pwm_set_gpio_level(xGPCCW, pwm);
	}
	printf("PWM %d, RPM %f\n", pwm, xRPM);
}





