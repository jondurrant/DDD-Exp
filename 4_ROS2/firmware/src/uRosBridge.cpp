/*
 * uRosBridge.cpp
 *
 * Bridge singleton object to manage all uRos comms
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "uRosBridge.h"

/***
 * Agent states
 */
enum states {
  WAITING_AGENT,    /**< WAITING_AGENT */
  AGENT_AVAILABLE,  /**< AGENT_AVAILABLE */
  AGENT_CONNECTED,  /**< AGENT_CONNECTED */
  AGENT_DISCONNECTED/**< AGENT_DISCONNECTED */
} ;

/***
 * Structure for the Publish queue
 */
struct PubCmd {
	rcl_publisher_t * publisher;
	void *msg;
	uRosEntities *entities;
	void *args;
};
typedef struct PubCmd PubCmd_t;

uRosBridge * uRosBridge::pSingleton = NULL;

/***
 * Get the uRos Bridge object
 * @return
 */
uRosBridge * uRosBridge::getInstance(){
	if (pSingleton == NULL){
		pSingleton = new uRosBridge();
	}
	return pSingleton;
}


uRosBridge::uRosBridge() {
	xPubQ = xQueueCreate( PUB_Q_LEN,
            sizeof(PubCmd_t) );

	if( xPubQ == NULL ){
		printf("ERROR uRosBridge create Queue failed\n");
	}

}

uRosBridge::~uRosBridge() {
	if( xPubQ != NULL ){
		vQueueDelete(xPubQ);
	}
}

/***
 * Run loop for the agent.
 */
void uRosBridge::run(){
	PubCmd_t cmd;
	uint readCount;

	uRosInit();
	xAllocator = rcl_get_default_allocator();
	states state = WAITING_AGENT;

	xMsg.data = 0;

	for(;;){
		switch (state) {
			case WAITING_AGENT:
			  state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
			  break;
			case AGENT_AVAILABLE:
			  createEntities();
			  state = AGENT_CONNECTED ;
			  break;
			case AGENT_CONNECTED:
			  state = pingAgent() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
			  if (state == AGENT_CONNECTED) {
				rclc_executor_spin_some(&xExecutor, RCL_MS_TO_NS(10));

				//Handle Pub queue
				if (xPubQ  != NULL){
					BaseType_t res = pdTRUE;
					readCount = 0;
					while (res == pdTRUE){
						res = xQueueReceive(
						   xPubQ,
						   &cmd,
						   0);

						if (res == pdTRUE){
							rcl_ret_t pubRet = rcl_publish(
									cmd.publisher,
									cmd.msg,
									NULL);
							if (RCL_RET_OK != pubRet) {
								printf("Queue Pub failed %d \n", pubRet);
								state = AGENT_DISCONNECTED;

								cmd.entities->pubComplete(
										cmd.msg,
										cmd.args,
										PubFailed
										);
								break;
							} else {
								cmd.entities->pubComplete(
									cmd.msg,
									cmd.args,
									PubOK
									);
							}

							readCount++;
							if (readCount > UROS_MAX_PUB_MSGS){
								break;
							}
						}

					}
				}


			  }
			  break;
			case AGENT_DISCONNECTED:
			  destroyEntities();
			  state = WAITING_AGENT;
			  break;
			default:
			  break;
		  }





		taskYIELD();
	}
}


/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE uRosBridge::getMaxStackSize(){
	return 1024;
}

/***
 * Initialise uROS by setting up allocator and transport
 */
void uRosBridge::uRosInit(){
	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = __freertos_allocate;
	freeRTOS_allocator.deallocate = __freertos_deallocate;
	freeRTOS_allocator.reallocate = __freertos_reallocate;
	freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	  printf("Error on default allocators (line %d)\n",__LINE__);
	  return;
	}

	rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_usb_transport_open,
		pico_usb_transport_close,
		pico_usb_transport_write,
		pico_usb_transport_read
	);

}

/***
 * Ping the uROS Agent
 * @return
 */
bool uRosBridge::pingAgent(){
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 100;
    const uint8_t attempts = 1;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
    	gpio_put(xLedPad, 0);
    	return false;
    } else {
    	gpio_put(xLedPad, 1);
    }
    return true;
}

/***
 * Set the Pad to use as a status LED
 * @param pad - GPIO PAD
 */
void uRosBridge::setLed(uint8_t pad){
	xLedPad = pad;
	gpio_init(xLedPad);
	gpio_set_dir(xLedPad, GPIO_OUT);
	gpio_put(xLedPad, 0);
}

/***
 * Create the Entities (Publishers)
 */
void uRosBridge::createEntities(){
	PubCmd_t cmd;
	xAllocator = rcl_get_default_allocator();

	rclc_support_init(&xSupport, 0, NULL, &xAllocator);

	rclc_node_init_default(&xNode, "pico_node", "", &xSupport);
	rclc_publisher_init_default(
		&xPublisher,
		&xNode,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"pico_count");

	rclc_timer_init_default(
		&xTimer,
		&xSupport,
		RCL_MS_TO_NS(1000),
		uRosBridge::timerCallback);


	uint handles = 1;
	uint count =   2;

	if (pURosEntities != NULL){
		pURosEntities->createEntities(&xNode, &xSupport);
		handles += pURosEntities->getHandles();
		count += pURosEntities->getCount();
	}


	printf("Created %d entities\n", count);

	rclc_executor_init(&xExecutor, &xSupport.context, handles, &xAllocator);
	rclc_executor_add_timer(&xExecutor, &xTimer);

	if (pURosEntities != NULL){
		pURosEntities->addToExecutor(&xExecutor);
	}


	//Sync Time
	if (RMW_RET_OK != rmw_uros_sync_session(5000)){
		printf("ERROR Time synk failed\n");
	}


	//Empty Queue as publishers have been regenerated
	if (xPubQ != NULL){
		BaseType_t res = pdTRUE;
		while (res == pdTRUE){
			res = xQueueReceive(
			   xPubQ,
			   &cmd,
			   0);

			if (res == pdTRUE){
				cmd.entities->pubComplete(
					cmd.msg,
					cmd.args,
					PubCleared
					);
			}

		}
	}

	printf("Entities Created\n");

}

/***
 * Destroy the entities
 */
void uRosBridge::destroyEntities(){
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&xSupport.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	rcl_publisher_fini(&xPublisher, &xNode);
	rcl_timer_fini(&xTimer);

	if (pURosEntities != NULL){
		pURosEntities->destroyEntities(&xNode, &xSupport);
	}

	rclc_executor_fini(&xExecutor);
	rcl_node_fini(&xNode);
	rclc_support_fini(&xSupport);

	printf("Entities Destroyed\n");
}

/***
 * Timer call back used for the pico_count
 * @param timer
 * @param last_call_time
 */
void uRosBridge::timerCallback(rcl_timer_t *timer, int64_t last_call_time){
	uRosBridge::getInstance()->pubCount();
}

/***
 * Publish the pico_count topic
 */
void uRosBridge::pubCount(){
    rcl_ret_t ret = rcl_publish(&xPublisher, &xMsg, NULL);
    //printf("Published %d\n", xMsg.data);
    xMsg.data++;
}

/***
 * set Ros Entities object. This is used to create and destroy publishers
 * @param mgr: pointer to the object managing the publishers
 */
void uRosBridge::setuRosEntities(uRosEntities *mgr){
	pURosEntities = mgr;
}

/***
 * Publish the msg using the publisher.
 * The msg should remain in scope until entities->pubComplete is called
 * @param publisher - ROS Publisher
 * @param msg - ROS Msg
 * @param entities - Entities object to get call back of pubComplete
 * @param arg - arguments to provide back to the pubComplete function
 * @return True if publish is queued ok
 */
bool uRosBridge::publish(rcl_publisher_t *publisher,
		void * msg,
		uRosEntities *entities,
		void *args){
	PubCmd_t cmd;

	if (xPubQ == NULL){
		return false;
	}

	cmd.publisher = publisher;
	cmd.msg = msg;
	cmd.entities = entities;
	cmd.args = args;

	BaseType_t res  = xQueueSendToBack(
		   xPubQ,
		   &cmd,
		   0
		   );

	if (res != pdTRUE){
		//printf("uRosBridge::publish Failed to send to buffer\n");
		entities->pubComplete(
				msg,
				args,
				PubFailed);
		return false;
	}
	return true;
}


/***
* Returns the ROS2 support pointer
* @return NULL if not connected
*/
rclc_support_t *uRosBridge::getSupport(){
	return &xSupport;
}
