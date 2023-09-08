/*
 * DDD.h
 *
 *  Created on: 7 Aug 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_DDD_H_
#define FIRMWARE_SRC_DDD_H_

#include "Agent.h"
#include "MotorsAgent.h"
#include "HCSR04Agent.h"
#include "uRosEntities.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
}

#include <Eigen/Core>
#include <Eigen/Geometry>

//#define TWIST_DEBUG

#define WHEEL_RADIUS   0.065
#define WHEEL_DEPTH    0.055
#define WHEELS_SEP     0.204
#define WHEELS_OFFSET  0.010 //To Y center of Robot

#define MAX_TWIST_TIME_MS 1500

struct DDDOdom {
	double x;
	double y;
	double a;
};

typedef struct DDDOdom DDDOdom_t;

class DDD : public Agent, public uRosEntities {
public:
	DDD();
	virtual ~DDD();

	void setMotorsAgent(MotorsAgent *p);

	void setHCSR04Agent(HCSR04Agent *p);


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


	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);


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

	void updateOdom();

	void publishOdom();

	void setupOdomMsg();
	void setupTwistMsg();

	void robotStop();

	MotorsAgent *pMotorsAgent = NULL;
	HCSR04Agent *pHCSR04Agent = NULL;

	DDDOdom_t xMotorsOdom;
	DDDOdom_t xDDDOdom;
	DDDOdom_t xDDDVelocity;

	uint32_t xLastVelocityTime = 0;

	rcl_publisher_t 			xPubOdom;
	nav_msgs__msg__Odometry 	xOdomMsg;

	rcl_subscription_t 			xSubTwist;
	uRosSubContext_t   			xSubTwistContext;
	geometry_msgs__msg__Twist 	xTwistMsg;

	uint32_t xLastTwistTimestamp = 0;

};

#endif /* FIRMWARE_SRC_DDD_H_ */
