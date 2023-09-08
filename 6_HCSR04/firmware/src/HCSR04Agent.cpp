/*
 * HCSR04Agent.cpp
 *
 *  Created on: 30 Aug 2023
 *      Author: jondurrant
 */

#include "HCSR04Agent.h"
#include <cstdio>
#include "uRosBridge.h"

HCSR04Agent::HCSR04Agent() {
	setupRange();

}

HCSR04Agent::~HCSR04Agent() {
	// TODO Auto-generated destructor stub
}


void HCSR04Agent::addSensor(uint trigger, const char * name){
	pSensors[xSensorCount] = new  DistanceSensor(pio0, xSensorCount, trigger);
	 if (!rosidl_runtime_c__String__assign(
			&xMsgsRange[xSensorCount].header.frame_id, name)){
			printf("ERROR: Range frameID assignment failed\n");
	}
	xSensorCount++;
}


/***
* Run loop for the agent.
*/
void HCSR04Agent::run(){
	for (;;){
		for (int i=0; i < xSensorCount; i++){
			pSensors[i]->TriggerRead();
			while (pSensors[i]->is_sensing){
				vTaskDelay(1);
			}
			publishRange(i);
		}
		vTaskDelay(200);
	}

}


/***
 * Get the static depth required in words
 * @return - words
 */
 configSTACK_DEPTH_TYPE HCSR04Agent::getMaxStackSize(){
	 return 256;
 }


 void HCSR04Agent::publishRange(uint sensorIndex){
	 //printf("Range(%u) %d cm\n",  sensorIndex, pSensors[sensorIndex]->distance);

	 if (xCount == 0){
		 return;
	 }

	 int64_t time = rmw_uros_epoch_nanos();
	 xMsgsRange[sensorIndex].header.stamp.sec = time / 1000000000;
	 xMsgsRange[sensorIndex].header.stamp.nanosec = time % 1000000000;
	 xMsgsRange[sensorIndex].range = (float) pSensors[sensorIndex]->distance / 100.0;
	 if ( xMsgsRange[sensorIndex].range > xMsgsRange[sensorIndex].max_range){
		 xMsgsRange[sensorIndex].range = xMsgsRange[sensorIndex].max_range;
	 }

	 if (!uRosBridge::getInstance()->publish(&xPubRange,
	 			&xMsgsRange[sensorIndex],
	 			this,
	 			NULL)){
	 		printf("Range Pub failed\n");
	 	}
 }


 void HCSR04Agent::setupRange(){
	 for (uint i =0 ; i < MAX_HCSR04_SENSORS; i++){
		 sensor_msgs__msg__Range__init(&xMsgsRange[i]);
		 xMsgsRange[i].radiation_type = 0;
		 xMsgsRange[i].field_of_view = 0.52;
		 xMsgsRange[i].min_range = 0.01;
		 xMsgsRange[i].max_range = 1.0;
	 }

 }



/***
 * Create the publishing entities
 * @param node
 * @param support
 */
 void HCSR04Agent::createEntities(rcl_node_t *node, rclc_support_t *support){
	 rclc_publisher_init_default(
	 			&xPubRange,
	 			node,
	 			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
	 			"/ddd/range");
	 xCount = 1;
 }

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
 void HCSR04Agent::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	 rcl_publisher_fini(&xPubRange, node);
	 xCount = 0;
 }

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
 uint HCSR04Agent::getCount(){
	 return xCount;
 }

/***
 * Return the number of handles needed by the executor
 * @return
 */
 uint HCSR04Agent::getHandles(){
	 return 0;
 }

