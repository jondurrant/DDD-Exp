/*
 * HCSR04Agent.h
 *
 *  Created on: 30 Aug 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_HCSR04AGENT_H_
#define FIRMWARE_SRC_HCSR04AGENT_H_

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Agent.h"
#include "uRosEntities.h"
#include "distance_sensor.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include "rosidl_runtime_c/string_functions.h"
#include <sensor_msgs/msg/range.h>
}


#define MAX_HCSR04_SENSORS 	4


class HCSR04Agent:  public Agent, public uRosEntities  {
public:
	HCSR04Agent();
	virtual ~HCSR04Agent();

	void addSensor(uint trigger, const char * name);

	/***
	 * Create the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Provide a count of the number of entities
	 * @return number of entities >=0
	 */
	virtual uint getCount();

	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
	void setupRange();
	void publishRange(uint sensorIndex);


	DistanceSensor* pSensors[MAX_HCSR04_SENSORS];
	uint xSensorCount = 0;

	rcl_publisher_t 								xPubRange;
	sensor_msgs__msg__Range 	xMsgsRange[MAX_HCSR04_SENSORS];

	uint xCount = 0;
};

#endif /* FIRMWARE_SRC_HCSR04AGENT_H_ */
