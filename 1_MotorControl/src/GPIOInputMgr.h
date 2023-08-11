/*
 * GPIOInputMgr.h
 *
 * Manage all GPIOInputs through a single class object to get around the single
 * callback function per core issue
 *
 *  Created on: 6 Jul 2022
 *      Author: jondurrant
 */

#ifndef PIR_PIRTIMER_SRC_GPIOMGR_H_
#define PIR_PIRTIMER_SRC_GPIOMGR_H_

#include "GPIOObserver.h"

#define NUM_GPIO 29

class GPIOInputMgr {
public:
	/***
	 * Constructor
	 */
	GPIOInputMgr();

	/***
	 * Destructor
	 */
	virtual ~GPIOInputMgr();

	/***
	 * Only one observer per GPIO Pad
	 * @param gpio - number of the GPIO Pad
	 * @param obs - Observer object
	 */
	void addObserver(uint gpio, GPIOObserver *obs);

	/***
	 * Get the GPIOInputMgr object
	 * Will create one if it does not exist
	 * @return
	 */
	static GPIOInputMgr *getMgr ();

private:
	// The list of observers, one per GPIO pad
	GPIOObserver *pObservers[NUM_GPIO];

	//Used for call back functions to find the object
	static GPIOInputMgr * pSelf;

	/***
	 * Call back function
	 * @param gpio
	 * @param events
	 */
	static void gpioCallback (uint gpio, uint32_t events);

	/***
	 * Handle a GPIO event by passing call to the correct object
	 * @param gpio = GPIO Pad number
	 * @param events - mask of the events
	 */
	void handleGPIO(uint gpio, uint32_t events);



};

#endif /* PIR_PIRTIMER_SRC_GPIOMGR_H_ */
