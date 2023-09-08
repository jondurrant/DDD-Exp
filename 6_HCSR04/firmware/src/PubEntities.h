/*
 * PubEntities.h
 *
 * Test agent to publish to two topics
 * 	pico_rnd: Random number on Topic
 * 	joint_states: Rotational join in Radians
 *
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#ifndef FREERTOSRECON_SRC_PUBENTITIES_H_
#define FREERTOSRECON_SRC_PUBENTITIES_H_

#include "Agent.h"
#include "uRosEntities.h"
#include <queue.h>
#include <math.h>

extern "C"{
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose_stamped.h>
}

#define MAX_SPEED 3.14

#define TABLE_CMDQ_LEN 10
enum TableCmdType { TABLE_SPEED, TABLE_POSE };
struct TableCmd {
	TableCmdType cmd;
	double speed;
	double angle;
	uint32_t sec;
	uint32_t nanosec;
};
typedef struct TableCmd TableCmd_t;

class PubEntities : public Agent, public uRosEntities {
public:
	PubEntities();
	virtual ~PubEntities();

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

	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);


	/***
	 * Minimum radians between two values.
	 * In CW or CCW direction
	 * @param a1 - radians -PI to +PI
	 * @param a2 - radians -PI to +PI
	 * @return
	 */
	double angleDiffAbs(double a1, double a2);

private:
	rcl_publisher_t    xPublisher;
	rcl_subscription_t xSubTwist;
	uRosSubContext_t   xSubTwistContext;
	rcl_subscription_t xSubPose;
	uRosSubContext_t   xSubPoseContext;

	geometry_msgs__msg__Twist 		xTwistMsg;
	geometry_msgs__msg__PoseStamped xPoseMsg;
	rcl_publisher_t xPubJoint;
	uint xCount = 0;

	double xSpeed = 0.0;

	//Queue of commands
	QueueHandle_t xCmdQ;
};

#endif /* FREERTOSRECON_SRC_PUBENTITIES_H_ */
