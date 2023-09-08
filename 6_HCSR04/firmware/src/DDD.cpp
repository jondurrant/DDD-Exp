/*
 * DDD.cpp
 *
 *  Created on: 7 Aug 2023
 *      Author: jondurrant
 */

#include "DDD.h"
#include <cstdio>
#include <cmath>
#include "uRosBridge.h"

using namespace Eigen;

DDD::DDD() {
	xMotorsOdom.x=0.0 - WHEELS_OFFSET;
	xMotorsOdom.y=0.0;
	xMotorsOdom.a=0.0;

	xDDDOdom.x=0.0;
	xDDDOdom.y=0.0;
	xDDDOdom.a=0.0;


}

DDD::~DDD() {
	// TODO Auto-generated destructor stub
}

void DDD::setMotorsAgent(MotorsAgent *p){
	pMotorsAgent = p;
}


/***
 * Run loop for the agent.
 */
void DDD::run(){
	setupOdomMsg();
	for (;;){
		if (pMotorsAgent != NULL){
			uint32_t now = to_ms_since_boot (get_absolute_time ());
			uint32_t timeSinceTwise = now - xLastTwistTimestamp;
			if (xLastTwistTimestamp > 0){
				if (timeSinceTwise > MAX_TWIST_TIME_MS){
					robotStop();
					xLastTwistTimestamp = 0;
					//printf("STOPPED due to twist lost\n");
				}
			}
			updateOdom();

			/*
			printf("X: %.3f Y: %.3f A: %.3f Deg: %.3f\n",
					xDDDOdom.x,
					xDDDOdom.y,
					xDDDOdom.a,
					xDDDOdom.a/M_PI * 180.0);
			 */

			publishOdom();

		}

		vTaskDelay(100);
	}
}
/* Old test of Odom
void DDD::run(){
	float leftCum = 0.0;
	float rightCum = 0.0;
	float mOdomX = 0.0;
	float mOdomY = 0.0;
	float mOdomA = 0.0;
	float odomX = 0.0;
	float odomY = 0.0;
	float odomA = 0.0;
	for (;;){

		if (pMotorsAgent != NULL){


			float l = pMotorsAgent->getMotor(0)->getDeltaRadians(false);
			float r = pMotorsAgent->getMotor(1)->getDeltaRadians(false);

			l=l*WHEEL_RADIUS;
			r=r*WHEEL_RADIUS* -1.0;
			leftCum  += l;
			rightCum += r;

			float avgDist = (r+l)/2.0;
			float angle = asin((r-l)/WHEELS_SEP);
			float deltaX = cos(angle)* avgDist;
			float deltaY = sin(angle)* avgDist;

			mOdomX += deltaX;
			mOdomY += deltaY;
			mOdomA += angle;

			odomX = mOdomX + (cos(angle) * WHEELS_OFFSET);
			odomY = mOdomY + (sin(angle) * WHEELS_OFFSET);
			odomA = mOdomA;


			printf("l: %.3f r: %0.3f dist: %.3f ang: %.3f "
				   "x: %.3f y: %.3f a: %.3f "
					"MX: %.3f MY: %.3f MA: %.3f "
					"X: %.3f, Y: %.3f A: %.3f\n",
					l, r, avgDist, angle,
					deltaX, deltaY, angle/M_PI * 180.0,
					mOdomX, mOdomY, mOdomA/M_PI * 180.0,
					odomX, odomY, odomA/M_PI * 180.0
					);

		}
		vTaskDelay(500);
	}
}
*/


/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE DDD::getMaxStackSize(){
	return 1024;
}

void DDD::updateOdom(){
	double l = pMotorsAgent->getMotor(0)->getDeltaRadians();
	double r = pMotorsAgent->getMotor(1)->getDeltaRadians() * (-1.0);

	//printf("Raw: l %.3f r %.3f\n", l, r);

	//l=l*WHEEL_RADIUS * 1.0;
	//r=r*WHEEL_RADIUS* -1.0;
	double diam = WHEEL_RADIUS * 2.0 * M_PI;
	l = diam * (l /(M_PI * 2.0) );
	r = diam * ( r/ (M_PI *2.0) );


	double avgDist = (r+l)/2.0;
	double angle = asin((r-l)/WHEELS_SEP);
	double deltaX = cos(angle)* avgDist;
	double deltaY = sin(angle)* avgDist;

	/*
	printf("Delta: l %.3f r %.3f avg %.3f ang %.3f X %.3f Y %.3f\n",
			l,
			r,
			avgDist,
			angle,
			deltaX,
			deltaY
			);
			*/

	xMotorsOdom.x += deltaX;
	xMotorsOdom.y += deltaY;
	xMotorsOdom.a += angle;

	xDDDOdom.x = xMotorsOdom.x + (cos(angle) * WHEELS_OFFSET);
	xDDDOdom.y = xMotorsOdom.y + (sin(angle) * WHEELS_OFFSET);
	xDDDOdom.a = xMotorsOdom.a;

	uint32_t now = to_ms_since_boot( get_absolute_time()    );
	double seconds = (double)(now - xLastVelocityTime) / 1000;
	xLastVelocityTime = now;
	xDDDVelocity.x = deltaX /seconds;
	xDDDVelocity.y = deltaY /seconds;
	xDDDVelocity.a = angle /seconds;

	/*
	printf("L: %.3f r: %0.3f dist: %.3f ang: %.3f "
			   "x: %.3f y: %.3f a: %.3f "
				"MX: %.3f MY: %.3f MA: %.3f "
				"X: %.3f, Y: %.3f A: %.3f\n",
				l, r, avgDist, angle,
				deltaX, deltaY, angle/M_PI * 180.0,
				xMotorsOdom.x, xMotorsOdom.y, xMotorsOdom.a/M_PI * 180.0,
				xDDDOdom.x, xDDDOdom.y, xDDDOdom.a/M_PI * 180.0
				);
	*/

}

void DDD::publishOdom(){
	//Update header
	int64_t time = rmw_uros_epoch_nanos();
	xOdomMsg.header.stamp.sec = time / 1000000000;
	xOdomMsg.header.stamp.nanosec = time % 1000000000;

	//POSE
	xOdomMsg.pose.pose.position.x = xDDDOdom.x;
	xOdomMsg.pose.pose.position.y = xDDDOdom.y;
	Quaterniond q;
	Matrix3d m;
	m = AngleAxisd(0.0, 		Vector3d::UnitX())
	  * AngleAxisd(0.0,  		Vector3d::UnitY())
	  * AngleAxisd(xDDDOdom.a, 	Vector3d::UnitZ());
	q = m;
	xOdomMsg.pose.pose.orientation.x = q.x();
	xOdomMsg.pose.pose.orientation.y = q.y();
	xOdomMsg.pose.pose.orientation.z = q.z();
	xOdomMsg.pose.pose.orientation.w = q.w();

	//TWIST
	xOdomMsg.twist.twist.linear.x 	= xDDDVelocity.x;
	xOdomMsg.twist.twist.linear.y 	= xDDDVelocity.y;
	xOdomMsg.twist.twist.angular.z 	= xDDDVelocity.a;


	if (!uRosBridge::getInstance()->publish(&xPubOdom,
			&xOdomMsg,
			this,
			NULL)){
		printf("Odom Pub failed\n");
	}

}

void DDD::setupOdomMsg(){
	nav_msgs__msg__Odometry__init(&xOdomMsg);
	if (!rosidl_runtime_c__String__assign(
			&xOdomMsg.header.frame_id, "odom")){
			printf("ERROR: Odom frameID assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(
			&xOdomMsg.child_frame_id, "base_link")){
			printf("ERROR: Odom frameID assignment failed\n");
	}

	//POSE
	xOdomMsg.pose.pose.position.x = 0.0;
	xOdomMsg.pose.pose.position.y = 0.0;
	xOdomMsg.pose.pose.position.z = 0.0;
	xOdomMsg.pose.pose.orientation.x = 0.0;
	xOdomMsg.pose.pose.orientation.y = 0.0;
	xOdomMsg.pose.pose.orientation.z = 0.0;
	xOdomMsg.pose.pose.orientation.w = 0.0;

	//TWIST
	xOdomMsg.twist.twist.linear.x = 0.0;
	xOdomMsg.twist.twist.linear.y = 0.0;
	xOdomMsg.twist.twist.linear.z = 0.0;
	xOdomMsg.twist.twist.angular.x = 0.0;
	xOdomMsg.twist.twist.angular.y = 0.0;
	xOdomMsg.twist.twist.angular.z = 0.0;

}

void DDD::setupTwistMsg(){
	geometry_msgs__msg__Twist__init(&xTwistMsg);
}


/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void DDD::createEntities(rcl_node_t *node, rclc_support_t *support){
	if (pMotorsAgent != NULL){
		pMotorsAgent->createEntities(node, support);
	}
	if (pHCSR04Agent != NULL){
		pHCSR04Agent->createEntities(node, support);
	}
	rclc_publisher_init_default(
			&xPubOdom,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
			"/ddd/odom");

	rclc_subscription_init_default(
			  &xSubTwist,
			  node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			  "/ddd/cmd_vel");
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void DDD::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	if (pMotorsAgent != NULL){
		pMotorsAgent->destroyEntities(node, support);
	}
	if (pHCSR04Agent != NULL){
		pHCSR04Agent->destroyEntities(node, support);
	}
	rcl_publisher_fini(&xPubOdom, node);
	rcl_subscription_fini(&xSubTwist, 	node);
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint DDD::getCount(){
	uint res = 2;
	if (pMotorsAgent != NULL){
		res += pMotorsAgent->getCount();
	}
	if (pHCSR04Agent != NULL){
		res += pHCSR04Agent->getCount();
	}
	return res;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint DDD::getHandles(){
	uint res = 1;
	if (pMotorsAgent != NULL){
		res += pMotorsAgent->getHandles();
	}
	if (pHCSR04Agent != NULL){
		res += pHCSR04Agent->getHandles();
	}
	return res;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void DDD::addToExecutor(rclc_executor_t *executor){
	if (pMotorsAgent != NULL){
		pMotorsAgent->addToExecutor(executor);
	}

	buildContext(&xSubTwistContext, NULL);
	rclc_executor_add_subscription_with_context(
			executor,
			&xSubTwist,
			&xTwistMsg,
			uRosEntities::subscriptionCallback,
			&xSubTwistContext,
			ON_NEW_DATA);
}


/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void DDD::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){

	if (pMotorsAgent == NULL){
		return;
	}

	if (context == &xSubTwistContext){
		geometry_msgs__msg__Twist * pTwistMsg = (geometry_msgs__msg__Twist *) msg;
		double circum = WHEEL_RADIUS * 2.0 * M_PI;

#ifdef TWIST_DEBUG
		printf("TWIST x: %.3f  rz: %.3f\n",
				pTwistMsg->linear.x,
				pTwistMsg->angular.z
				);
#endif //TWIST_DEBUG

		xLastTwistTimestamp = to_ms_since_boot (get_absolute_time ());

		//Stop
		if (pTwistMsg->linear.x == 0.0){
			//Have not move linearly to turn
			pMotorsAgent->setSpeedRadPS(0, 0.0, true);
			pMotorsAgent->setSpeedRadPS(1, 0.0, false);
			return;
		}

		// FWD and Backwards
		if (pTwistMsg->angular.z == 0.0){
			double rps = (pTwistMsg->linear.x / circum) * (2 * M_PI);
			bool cw = true;
			if (rps < 0.0){
				cw = false;
				rps = rps * -1;
			}
			pMotorsAgent->setSpeedRadPS(0, rps, !cw);
			pMotorsAgent->setSpeedRadPS(1, rps, cw);

#ifdef TWIST_DEBUG
			printf("LINEAR TWIST %.3f mps becomes %.3f Rad ps\n",
					pTwistMsg->linear.x,
					rps
					);
#endif //TWIST_DEBUG
		} else {
			//ARC
			bool fwd = (pTwistMsg->linear.x > 0.0);
			bool cw = (pTwistMsg->angular.z > 0.0);
			double a = fabs(pTwistMsg->angular.z);
			double arc = a/ (M_PI * 2);
			double fullCircleCircum = (pTwistMsg->linear.x / arc);
			double radius = fullCircleCircum / ( 2.0 * M_PI);

			double speedA = (radius + WHEELS_SEP/4) * (2 * M_PI) * arc;
			double speedB = (radius - WHEELS_SEP/4) * (2 * M_PI) * arc;

			double rpsA = (speedA / circum) * (2 * M_PI);
			double rpsB = (speedB / circum) * (2 * M_PI);

			if (fwd){
				if (!cw){
					pMotorsAgent->setSpeedRadPS(0, rpsA, !fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsB,  fwd);
				} else {
					pMotorsAgent->setSpeedRadPS(0, rpsB, !fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsA,  fwd);
				}
			} else {
				if (cw){
					pMotorsAgent->setSpeedRadPS(0, rpsA,  fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsB, !fwd);
				} else {
					pMotorsAgent->setSpeedRadPS(0, rpsB,  fwd);
					pMotorsAgent->setSpeedRadPS(1, rpsA, !fwd);
				}
			}

#ifdef TWIST_DEBUG
			printf("ROTATE TWIST %.3f mps at %.3f rad ps "
					"becomes %.3f and %.3f Rad ps\n",
					pTwistMsg->linear.x,
					pTwistMsg->angular.z,
					rpsA,
					rpsB
					);
			printf("ROTATE Detail: Radius %.3f Full Circum %.3f Arc %.3f speed A %.3f B %.3f\n",
					radius,
					fullCircleCircum,
					arc,
					speedA,
					speedB);
#endif //TWIST_DEBUG

		}


	}
}


void DDD::robotStop(){
	xLastTwistTimestamp = to_ms_since_boot (get_absolute_time ());

	if (pMotorsAgent != NULL){
		pMotorsAgent->setSpeedRadPS(0, 0.0, true);
		pMotorsAgent->setSpeedRadPS(1, 0.0, false);
	}
}

void DDD::setHCSR04Agent(HCSR04Agent *p){
	pHCSR04Agent = p;
}

