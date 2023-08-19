/*
 * GPIOMgr.cpp
 *
 *  Created on: 6 Jul 2022
 *      Author: jondurrant
 */

#include "GPIOInputMgr.h"
#include <cstddef>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/gpio.h"
#include <stdio.h>

GPIOInputMgr *GPIOInputMgr::pSelf = NULL;

/***
 * Constructor
 */
GPIOInputMgr::GPIOInputMgr() {
	for (uint8_t i=0; i < NUM_GPIO; i++){
		pObservers[i] = NULL;
	}
	GPIOInputMgr::pSelf = this;
}

/***
 * Destructor
 */
GPIOInputMgr::~GPIOInputMgr() {
	//NOP
}

/***
 * Only one observer per GPIO Pad
 * @param gpio - number of the GPIO Pad
 * @param obs - Observer object
 */
void GPIOInputMgr::addObserver(uint gpio, GPIOObserver *obs){
	//Initialise the GPIO pin
	gpio_init(gpio);
	gpio_set_dir(gpio, GPIO_IN);

	//Setup Interrupt
	gpio_set_irq_enabled_with_callback(gpio,
		GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
		true,
		GPIOInputMgr::gpioCallback
	);

	pObservers[gpio] = obs;
}

/***
 * Call back function
 * @param gpio
 * @param events
 */
void GPIOInputMgr::gpioCallback (uint gpio, uint32_t events){
	if (GPIOInputMgr::pSelf != NULL){
		GPIOInputMgr::pSelf->handleGPIO(gpio, events);
	}
}

/***
 * Handle a GPIO event by passing call to the correct object
 * @param gpio = GPIO Pad number
 * @param events - mask of the events
 */
void GPIOInputMgr::handleGPIO(uint gpio, uint32_t events){
	if (pObservers[gpio] != NULL){
		pObservers[gpio]->handleGPIO(gpio, events);
	}
}

/***
 * Get the GPIOInputMgr object
 * Will create one if it does not exist
 * @return
 */
GPIOInputMgr *GPIOInputMgr::getMgr (){
	if (GPIOInputMgr::pSelf == NULL){
		GPIOInputMgr *m = new GPIOInputMgr();
	}

	return GPIOInputMgr::pSelf;
}
