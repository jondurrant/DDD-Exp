/*
 * MotorsAgent.cpp
 *
 *  Created on: 6 Aug 2023
 *      Author: jondurrant
 */

#include "MotorsAgent.h"

#include "uRosBridge.h"

#include <inttypes.h>
#include <cmath>


MotorsAgent::MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		pMotors[i] = NULL;
	}
}

MotorsAgent::~MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			delete pMotors[i];
		}
	}
}

/***
 * Add Motor
 * @param index - Index of the Motor
 * @param gpCW - CW power terminal on Motor
 * @param gpCCW - CCW power terminal on Motor
 * @param gpA - RotEnc input A
 * @param gpB - RotEnc input B
 */
void MotorsAgent::addMotor(uint index,
		uint8_t gpCW, uint8_t gpCCW,
		uint8_t gpA, uint8_t gpB){
	if (index < NUM_MOTORS){
		pMotors[index] = new MotorPID(gpCW, gpCCW, gpA, gpB);
	}
}

/***
 * Configure PID for motor
 * @param index - of the motor
 * @param kP
 * @param kI
 * @param kD
 */
void MotorsAgent::configPID(uint index,
		float kP, float kI, float kD){
	if (pMotors[index] != NULL){
		pMotors[index]->configPID(kP, kI, kD);
	}
}

/***
 * Configure PID for all the motors
 * @param kP
 * @param kI
 * @param kD
 */
void MotorsAgent::configAllPID(float kP, float kI, float kD){
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			pMotors[i]->configPID(kP, kI, kD);
		}
	}
}


/***
 * Set the speed of the motor to be controlled
 * @param index of the motor
 * @param rpm rev per minute
 * @param cw direction - true if clockwise
 */
void MotorsAgent::setSpeedRPM(uint index,
		float rpm, bool cw){
	if (pMotors[index] != NULL){
		pMotors[index]->setSpeedRPM(rpm, cw);
	}
}

/***
 * Set the speed of the motor to be controlled
 * @param index of the motor
 * @param rps radians per second
 * @param cw direction - true if clockwise
 */
void MotorsAgent::setSpeedRadPS(uint index,
		float rps, bool cw){
	if (pMotors[index] != NULL){
		if (rps >= 0.0){
			pMotors[index]->setSpeedRadPS(rps, cw);
		} else {
			pMotors[index]->setSpeedRadPS(fabs(rps), !cw);
		}
	}
}

/***
 * Run loop for the agent.
 */
void MotorsAgent::run(){

	initJointState();

	for (;;){
		for (uint i=0; i < NUM_MOTORS; i++){
			if (pMotors[i] != NULL){
				float err = pMotors[i]->doPID();

				//printf("Error(%u) = %.3f\n",i, err);

				//printf("%u Rad %f\n", i,
				//		pMotors[i]->getRadians());
			}
		}

		pubJointState();

		vTaskDelay(200);
	}
}


/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE MotorsAgent::getMaxStackSize(){
	return 1024;
}


/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void MotorsAgent::createEntities(
		rcl_node_t *node,
		rclc_support_t *support){
	rclc_publisher_init_default(
		&xPubJoint,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"joint_states");
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void MotorsAgent::destroyEntities(
		rcl_node_t *node,
		rclc_support_t *support){

	rcl_publisher_fini(&xPubJoint, node);
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint MotorsAgent::getCount(){
	return 0;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint MotorsAgent::getHandles(){
	return 1;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void MotorsAgent::addToExecutor(rclc_executor_t *executor){
	//NOP
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void MotorsAgent::handleSubscriptionMsg(
		const void* msg,
		uRosSubContext_t* context){
	//NOP
}


void MotorsAgent::initJointState(){
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	char name[32];

	//Possition
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.position, NUM_MOTORS);
	xJointStateMsg.position.data[0] = 0.0;
	xJointStateMsg.position.size = NUM_MOTORS;
	xJointStateMsg.position.capacity = NUM_MOTORS;

	//Velocity
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.velocity, NUM_MOTORS);
	xJointStateMsg.velocity.data[0] = 0.0;
	xJointStateMsg.velocity.size = NUM_MOTORS;
	xJointStateMsg.velocity.capacity = NUM_MOTORS;

	//Name
	rosidl_runtime_c__String__Sequence__init(
			&xJointStateMsg.name, NUM_MOTORS);
	for (uint i=0; i < NUM_MOTORS; i++){
		sprintf(name, "motor_%u", i);
		if (!rosidl_runtime_c__String__assign(
				&xJointStateMsg.name.data[i], name)){
			printf("ERROR: Joined assignment failed\n");
		}
	}
	xJointStateMsg.name.size=NUM_MOTORS;
	xJointStateMsg.name.capacity=NUM_MOTORS;
}


void MotorsAgent::pubJointState(){
	//Populate the Joint possition message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;

	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL) {
			xJointStateMsg.position.data[i] =
					pMotors[i]->getRadians() - M_PI;

			xJointStateMsg.velocity.data[i] =
					pMotors[i]->getAvgRadPerSec();
		}
	}
	if (!uRosBridge::getInstance()->publish(&xPubJoint,
			&xJointStateMsg,
			this,
			NULL)){
		printf("Joint Pub failed\n");
	}
}

/***
 * Return specific motor or NULL if none
 * @param index
 * @return
 */
MotorPID * MotorsAgent::getMotor(uint index){
	if (index >= NUM_MOTORS){
		return NULL;
	}
	return pMotors[index];
}


