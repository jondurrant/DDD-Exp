/*
 * uRosBridge.h
 *
 * Bridge singleton object to manage all uRos comms
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#ifndef FREERTOSTEST_SRC_UROSBRIDGE_H_
#define FREERTOSTEST_SRC_UROSBRIDGE_H_


#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Agent.h"
#include "uRosEntities.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_usb_transports.h"

#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"

#include "freertos_allocators.h"
}


#define PUB_Q_LEN 			10
#define UROS_MAX_PUB_MSGS	5

class uRosBridge : public Agent {
public:

	/***
	 * Get the uRos Bridge object
	 * @return
	 */
	static uRosBridge * getInstance();

	/***
	 * Set the Pad to use as a status LED
	 * @param pad - GPIO PAD
	 */
	void setLed(uint8_t pad);

	/***
	 * set Ros Entities object. This is used to create and destroy publishers
	 * @param mgr: pointer to the object managing the publishers
	 */
	void setuRosEntities(uRosEntities *mgr);

	/***
	 * Publish the msg using the publisher.
	 * The msg should remain in scope until entities->pubComplete is called
	 * @param publisher - ROS Publisher
	 * @param msg - ROS Msg
	 * @param entities - Entities object to get call back of pubComplete
	 * @param arg - arguments to provide back to the pubComplete function
	 * @return True if publish is queued ok
	 */
	bool publish(rcl_publisher_t *publisher,
			 void * msg,
			 uRosEntities *entities,
			 void *arg
			 );


	/***
	 * Returns the ROS2 support pointer
	 * @return NULL if not connected
	 */
	rclc_support_t *getSupport();

private:



	uRosBridge();
	virtual ~uRosBridge();

	/***
	 * Initialise uROS by setting up allocator and transport
	 */
	void uRosInit();

	/***
	 * Ping the uROS Agent
	 * @return
	 */
	bool pingAgent();

	/***
	 * Create the Entities (Publishers)
	 */
	void createEntities();

	/***
	 * Destroy the entities
	 */
	void destroyEntities();

	/***
	 * Timer call back used for the pico_count
	 * @param timer
	 * @param last_call_time
	 */
	static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);


	static uRosBridge * pSingleton;
	rcl_publisher_t xPublisher;
	std_msgs__msg__Int32 xMsg;
	rcl_timer_t xTimer;
	rcl_node_t xNode;
	rcl_allocator_t xAllocator;
	rclc_support_t xSupport;
	rclc_executor_t xExecutor;

	uint8_t xLedPad = 0;

	uRosEntities * pURosEntities = NULL;

	QueueHandle_t xPubQ = NULL;


protected:
	/***
	 * Publish the pico_count topic
	 */
	void pubCount();

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();
};

#endif /* FREERTOSTEST_SRC_UROSBRIDGE_H_ */
