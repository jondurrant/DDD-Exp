/*
 * uRosEntities.cpp
 *
 * `Abstrac
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "uRosEntities.h"

uRosEntities::uRosEntities() {
	// TODO Auto-generated constructor stub

}

uRosEntities::~uRosEntities() {
	// TODO Auto-generated destructor stub
}

/***
 * Call back on a publish to show msg has been completed.
 * Can be used to free up allocated memory
 * @param msg - ROS Msg
 * @param args - Arguments passed into the publish step
 * @param status -
 */
void uRosEntities::pubComplete(
		void *msg,
		void *args,
		uRosPubStatus status){
	//NOP
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint uRosEntities::getHandles(){
	return 0;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void uRosEntities::addToExecutor(rclc_executor_t *executor){
	//NOP
}


/***
 * Call back function that handles the msg
 * @param msg
 * @param context
 */
void uRosEntities::subscriptionCallback(const void* msg, void* context){
	uRosSubContext_t *con = (uRosSubContext_t *) context;
	con->subHandler->handleSubscriptionMsg(msg, con);
}


/***
 * Build the callback context
 * @param context
 */
void uRosEntities::buildContext(uRosSubContext_t *context, void *localContext){
	context->subHandler = this;
	context->localContext = localContext;
}


/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void uRosEntities::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){
	//NOP
}

