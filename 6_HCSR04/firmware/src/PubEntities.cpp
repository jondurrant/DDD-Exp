/*
 * PubEntities.cpp
 *
 * Test agent to publish to two topics
 * 	pico_rnd: Random number on Topic
 * 	joint_states: Rotational join in Radians
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "PubEntities.h"
#include "pico/rand.h"
#include "uRosBridge.h"

#include <inttypes.h>

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
}

#include <cmath>

PubEntities::PubEntities() {
	geometry_msgs__msg__Twist__init(&xTwistMsg);
	geometry_msgs__msg__PoseStamped__init(&xPoseMsg);

	xCmdQ = xQueueCreate( TABLE_CMDQ_LEN, sizeof(TableCmd_t));
	if (xCmdQ == NULL){
		printf("ERROR: Unable to create Queue\n");
	}
}

PubEntities::~PubEntities() {
	geometry_msgs__msg__Twist__fini(&xTwistMsg);
	geometry_msgs__msg__PoseStamped__fini(&xPoseMsg);
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE PubEntities::getMaxStackSize(){
	return 256;
}


/***
 * Run loop for the agent.
 */
void PubEntities::run(){
	BaseType_t res;
	TableCmd_t cmd;
	uint32_t lastMs, nowMs;
	std_msgs__msg__Int32 msg;
	double targetAngle = 10;
	double nextAngle = 0;

	sensor_msgs__msg__JointState joint_state_msg;
	struct timespec ts;
	clockid_t id;
	double a= 0.0;
	const char *jointName = "post_to_table";


	//Setup the joint state msg
	sensor_msgs__msg__JointState__init(&joint_state_msg);
	rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, 1);
	joint_state_msg.position.data[0] = a;
	joint_state_msg.position.size = 1;
	joint_state_msg.position.capacity = 1;
	rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 1);
	if (!rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], jointName)){
		printf("ERROR: Joined assignment failed\n");
	}
	joint_state_msg.name.size=1;
	joint_state_msg.name.capacity=1;


	lastMs = nowMs = to_ms_since_boot(get_absolute_time());

	for (;;){
		if (xCount != 0){
			//CMD QUEUE Management
			res = xQueueReceive(xCmdQ, (void *)&cmd, 0);
			if (res == pdTRUE){
				if (cmd.cmd == TABLE_SPEED){
					xSpeed = cmd.speed;
				}
				if (cmd.cmd == TABLE_POSE){
					double cenCur = a + M_PI;
					targetAngle = fmod(cmd.angle, M_PI);

					//Calc distance to tracel Clockwise and Counter Clockwise
					double cenTar = targetAngle + M_PI;
					double cw = fmod(cenTar - cenCur, (M_PI * 2));
					if (cw < 0){
						cw = (M_PI *2) + cw;
					}
					double ccw = (M_PI *2) - cw;

					//Calc difference between timestamp and now
					int64_t now = rmw_uros_epoch_nanos();
					uint32_t sec = now / 1000000000;
					uint32_t nanosec = now % 1000000000;
					double dif = cmd.sec - sec;
					double ndif = cmd.nanosec % 1000000000;
					ndif = (ndif - nanosec) / 1000.0 / 1000.0 /1000.0;
					//If timestamp in past the 0 time
					if (dif < 0.0){
						dif = 0.0;
					}
					if ((dif == 0.0) && (ndif < 0.0)){
						dif = 0.0;
					} else {
						dif = dif + ndif;
					}

					//Calc speed
					if (cw < ccw){
						if (dif == 0.0){
							xSpeed = MAX_SPEED ;
						} else {
							xSpeed = (cw / dif);
							printf("Move from %f to %f (distance %f) at %f rads per sec Time to move %f\n",
									a, targetAngle, cw, xSpeed, dif);

						}
					} else {
						if (dif == 0.0){
							xSpeed = MAX_SPEED * -1.0;
						} else {
							xSpeed = (ccw / dif) * -1.0;
							printf("Move from %f to %f (distance %f) at %f rads per sec Time to move %f\n",
									a, targetAngle, ccw, xSpeed, dif);
						}
					}



				}
			}



			//Populate the Int32 message for pico_rnd
			msg.data = get_rand_32() % 0x7FFFFFFF;
			if (!uRosBridge::getInstance()->publish(
					&xPublisher,
					&msg,
					this,
					NULL)){
				printf("RND Pub failed\n");
			}

			// Calc radian for Rotating and accelerating turn table
			nowMs = to_ms_since_boot(get_absolute_time());
			uint32_t difMs = nowMs - lastMs;
			lastMs = nowMs;
			nextAngle = (xSpeed / 1000 * difMs) + a;
			nextAngle = fmod(nextAngle + M_PI, M_PI*2.0)- M_PI;

			if (xSpeed != 0.0){
				double difNext = angleDiffAbs(nextAngle, targetAngle);
				double difPre  = angleDiffAbs(a, targetAngle);

				/*DEBUG for mod arithmetic
				printf("At %f Next %f To %f Dif %f %f\n",
						a,
						nextAngle,
						targetAngle,
						difPre,
						difNext);
						*/
				if ((difNext >= difPre) && (targetAngle <= M_PI)){
					printf("\nStopping at %f before %f target %f\n",
							a,
							nextAngle,
							targetAngle
							);

					xSpeed = 0.0;
					targetAngle = 10.0;

				} else {
					a = nextAngle;
				}
			}

			//Populate the Joint possition message
			int64_t time = rmw_uros_epoch_nanos();

		    joint_state_msg.header.stamp.sec = time / 1000000000;
		    joint_state_msg.header.stamp.nanosec = time % 1000000000;
		    joint_state_msg.position.data[0] = a;
		    if (!uRosBridge::getInstance()->publish(&xPubJoint,
		    		&joint_state_msg,
					this,
					NULL)){
				printf("Joint Pub failed\n");
			}

		}
		vTaskDelay(100);
	}
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint PubEntities::getCount(){
	return xCount;
}

/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void PubEntities::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
			&xPublisher,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"pico_rnd");

	rclc_publisher_init_default(
			&xPubJoint,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			"joint_states");

	rclc_subscription_init_default(
		  &xSubTwist,
		  node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		  "cmd_vel");

	rclc_subscription_init_default(
		  &xSubPose,
		  node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		  "do_pose");

	xCount = 3;
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void PubEntities::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPublisher, 	node);
	rcl_publisher_fini(&xPubJoint, 		node);
	rcl_subscription_fini(&xSubTwist, 	node);
	rcl_subscription_fini(&xSubPose, 	node);
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint PubEntities::getHandles(){
	return 2;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void PubEntities::addToExecutor(rclc_executor_t *executor){
	buildContext(&xSubTwistContext, NULL);
	buildContext(&xSubPoseContext, NULL);

	rclc_executor_add_subscription_with_context(
		executor,
		&xSubTwist,
		&xTwistMsg,
		uRosEntities::subscriptionCallback,
		&xSubTwistContext,
		ON_NEW_DATA);

	rclc_executor_add_subscription_with_context(
		executor,
		&xSubPose,
		&xPoseMsg,
		uRosEntities::subscriptionCallback,
		&xSubPoseContext,
		ON_NEW_DATA);
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void PubEntities::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){
	BaseType_t res;
	TableCmd_t cmd;

	if (context == &xSubTwistContext){
		geometry_msgs__msg__Twist * pTwistMsg = (geometry_msgs__msg__Twist *) msg;

		//xSpeed = pTwistMsg->angular.z;

		cmd.cmd = TABLE_SPEED;
		cmd.speed = pTwistMsg->angular.z;

		if (xCmdQ != NULL){
			res = xQueueSendToBack(xCmdQ, (void *)&cmd, 0);
			if (res != pdTRUE){
				printf("WARNING: Queue is full\n");
			}
		}
	}

	if (context == &xSubPoseContext){
		geometry_msgs__msg__PoseStamped *pPoseMsg;
		pPoseMsg = (geometry_msgs__msg__PoseStamped *) msg;

		geometry_msgs__msg__Quaternion *q = &pPoseMsg->pose.orientation;
		double siny_cosp = 2 * (q->w * q->z + q->x * q->y);
		double cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
		double a = std::atan2(siny_cosp, cosy_cosp);

		//printf("Q %f %f %f %f\n", q->x, q->y, q->z, q->w );
		//printf("Req Pose %f\n", a);

		cmd.cmd = TABLE_POSE;
		cmd.angle = a;
		cmd.sec = pPoseMsg->header.stamp.sec;
		cmd.nanosec = pPoseMsg->header.stamp.nanosec;
		if (xCmdQ != NULL){
			res = xQueueSendToBack(xCmdQ, (void *)&cmd, 0);
			if (res != pdTRUE){
				printf("WARNING: Queue is full\n");
			}
		}
	}
}

/***
 * Minimum radians between two values.
 * In CW or CCW direction
 * @param a1 - radians -PI to +PI
 * @param a2 - radians -PI to +PI
 * @return
 */
double PubEntities::angleDiffAbs(double a1, double a2){
	double cenA1 = a1 + M_PI;
	double cenA2 = a2 + M_PI;

	double cw = cenA1 - cenA2;
	if (cw < 0.0){
		cw = cw * -1.0;
	}
	double ccw =  M_PI * 2.0 - cw;
	if (ccw < 0.0){
		ccw = ccw * -1.0;
	}
	if (ccw < cw){
		return ccw;
	}
	return cw;
}


