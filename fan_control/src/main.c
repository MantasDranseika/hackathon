#include <zephyr.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>i

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>
#include <unistd.h>
#include <time.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// controller subscriber 
rcl_subscription_t fanOn_subscriber;
std_msgs__msg__Int32 fanOn;

// temperature publisher
rcl_publisher_t publisher;
std_msgs__msg__Int32 msgTemp;

int maxTemp = 40;
int minTemp = 10;

// subscriber 
void fanOn_subscriber_callback(const void * msgIn)
{
	const std_msgs__msg__Int32 * msgOut = (const std_msgs__msg__Int32 *)msgIn;
  printf("Reveived command %d \n", msgOut->data);
}

// publisher
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{	
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msgTemp, NULL));
		if(msgTemp.data < maxTemp){
			msgTemp.data++;
		}
		else {
			msgTemp.data = minTemp;
		}
	}
}

void main(void)
{	
// Set custom transports
	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *) &default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read
	);

	// Init micro-ROS
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "zephyr_fan_control", "", &support));

  // create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"zephyr_fan_control"));

  // create subscriber
	RCCHECK(rclc_subscription_init_best_effort(
    &fanOn_subscriber, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
    "fan_brain"));

	// create timer
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &fanOn_subscriber, &fanOn, &fanOn_subscriber_callback, ON_NEW_DATA));

	msgTemp.data = 0;
 	
	while(1){
    	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		  usleep(100000);
	}

	RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&fanOn_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
