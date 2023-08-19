/*
 * GPIOObserver.h
 *
 * Observer event on a GPIO Pad
 *
 *  Created on: 6 Jul 2022
 *      Author: jondurrant
 */

#ifndef PIR_PIRTIMER_SRC_GPIOOBSERVER_H_
#define PIR_PIRTIMER_SRC_GPIOOBSERVER_H_

#include "pico/stdlib.h"
#include <cstddef>
#include <stdlib.h>


class GPIOObserver {
public:
	GPIOObserver();
	virtual ~GPIOObserver();

	/***
	 * handle GPIO  events
	 * @param gpio - GPIO number
	 * @param events - Event
	 */
	virtual void handleGPIO(uint gpio, uint32_t events)=0;

};

#endif /* PIR_PIRTIMER_SRC_GPIOOBSERVER_H_ */
