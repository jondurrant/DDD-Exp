/*
 * PWMManager.h
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#ifndef PWMDRIVE_SRC_PWMMANAGER_H_
#define PWMDRIVE_SRC_PWMMANAGER_H_

#define GP_STOP 2
#define GP_UP   3
#define GP_DOWN 4

#define RPM_INC 0.1;


#include "SwitchObserver.h"
#include "SwitchMgr.h"

class PWMManager : public SwitchObserver{
public:
	PWMManager(uint8_t gpCW, uint8_t gpCCW);
	virtual ~PWMManager();

	/***
	 * Handle a short press from the switch
	 * @param gp - GPIO number of the switch
	 */
	virtual void handleShortPress(uint8_t gp);

	/***
	 * Handle a short press from the switch
	 * @param gp - GPIO number of the switch
	 */
	virtual void handleLongPress(uint8_t gp);

	/***
	 * Handle a  press from the switch
	 * @param gp - GPIO number of the switch
	 */
	virtual void handlePress(uint8_t gp);

	void setSpeed();

private:
	uint8_t xGPCW=0;
	uint8_t xGPCCW=0;
	SwitchMgr xSwitches;

	float xRPM = 0.0;
};

#endif /* PWMDRIVE_SRC_PWMMANAGER_H_ */
