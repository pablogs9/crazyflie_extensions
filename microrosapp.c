/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <geometry_msgs/msg/point32.h>

#include <rcutils/allocator.h>

#include <stdio.h> //printf
#include <string.h> //strcmp
#include <stdlib.h> //atoi
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"

#include "microrosapp.h"

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

void appMain(){ 
    absoluteUsedMemory = 0;
    usedMemory = 0;

    vTaskDelay(2000);
    int radio_connected = logGetVarId("radio", "isConnected");
    while(!logGetUint(radio_connected)) vTaskDelay(100);
    DEBUG_PRINT("Radio connected\r\n");

    // ####################### MICROROS INIT #######################

    DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
    vTaskDelay(50);

    rcl_context_t      context;
    rcl_init_options_t init_options;
    rcl_ret_t          rc;
    init_options = rcl_get_zero_initialized_init_options();

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = __crazyflie_allocate;
    freeRTOS_allocator.deallocate = __crazyflie_deallocate;
    freeRTOS_allocator.reallocate = __crazyflie_reallocate;
    freeRTOS_allocator.zero_allocate = __crazyflie_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        DEBUG_PRINT("rcutils_set_default_allocator error\r\n");
        vTaskSuspend( NULL );
    }

    rc = rcl_init_options_init(&init_options, rcutils_get_default_allocator());
    
    if (rc != RCL_RET_OK) {
        DEBUG_PRINT("rcl_init_options_init error\r\n");
        vTaskSuspend( NULL );
    }

    context = rcl_get_zero_initialized_context();

    rc = rcl_init(0, NULL, &init_options, &context);
    if (rc != RCL_RET_OK) {
        vTaskSuspend( NULL );
        DEBUG_PRINT("rcl_init error\r\n");
    }

    rc = rcl_init_options_fini(&init_options);

    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();

    rc = rcl_node_init(&node, "crazyflie_node", "", &context, &node_ops);

    // Create publisher 1
    const char* drone_odom = "/drone/odometry";

    rcl_publisher_t pub_odom        = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * pub_type_support_odom = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
    rcl_publisher_options_t pub_opt_odom = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &pub_odom,
        &node,
        pub_type_support_odom,
        drone_odom,
        &pub_opt_odom);

    // Create publisher 2
    const char* drone_attitude = "/drone/attitude";

    rcl_publisher_t pub_attitude        = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * pub_type_support_att = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32);
    rcl_publisher_options_t pub_opt_att = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &pub_attitude,
        &node,
        pub_type_support_att,
        drone_attitude,
        &pub_opt_att);

    geometry_msgs__msg__Point32 pose;
    geometry_msgs__msg__Point32__init(&pose);
    geometry_msgs__msg__Point32 odom;
    geometry_msgs__msg__Point32__init(&odom);


  // ####################### MAIN LOOP #######################

    //Get pitch, roll and yaw value
    pitchid = logGetVarId("stateEstimate", "pitch");
    rollid = logGetVarId("stateEstimate", "roll");
    yawid = logGetVarId("stateEstimate", "yaw");

    //Get X,Y and Z value
    Xid = logGetVarId("stateEstimate", "x");
    Yid = logGetVarId("stateEstimate", "y");
    Zid = logGetVarId("stateEstimate", "z");

    DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
    DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
    DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

    while(1){
      float pitch = logGetFloat(pitchid);
      float roll  = logGetFloat(rollid);
      float yaw   = logGetFloat(yawid);
      float x     = logGetFloat(Xid);
      float y     = logGetFloat(Yid);
      float z     = logGetFloat(Zid);

      pose.x = pitch;
      pose.y = roll;
      pose.z = yaw;

      odom.x = x;
      odom.y = y;
      odom.z = z;

      rc = rcl_publish( &pub_odom, (const void *) &odom, NULL);
      rc = rcl_publish( &pub_attitude, (const void *) &pose, NULL);

      vTaskDelay(100/portTICK_RATE_MS);
    }
    
    vTaskSuspend( NULL );
}