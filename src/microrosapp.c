/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <geometry_msgs/msg/point32.h>
#include "example_interfaces/srv/add_two_ints.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

#include <rcutils/allocator.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include <time.h>

#include "microrosapp.h"

#define RCCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); vTaskSuspend( NULL );}
#define RCSOFTCHECK(msg) if((rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

float sign(float x){   
    return (x >= 0) ? 1.0 : -1.0;
}


typedef struct quaternion{
    double x;
    double y;
    double z;
    double w;
} quaternion;

void euler_to_quaternion(float roll, float pitch, float yaw, quaternion * q){
    q->x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    q->y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    q->z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    q->w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
}

void appMain(){ 
    absoluteUsedMemory = 0;
    usedMemory = 0;

    vTaskDelay(2000);
    int radio_connected = logGetVarId("radio", "isConnected");
    while(!logGetUint(radio_connected)) vTaskDelay(100);
    DEBUG_PRINT("Radio connected\n");

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
        DEBUG_PRINT("Error on default allocators (line %d)\n",__LINE__); 
        vTaskSuspend( NULL );
    }

    rc = rcl_init_options_init(&init_options, rcutils_get_default_allocator());
    RCCHECK()

    context = rcl_get_zero_initialized_context();

    rc = rcl_init(0, NULL, &init_options, &context);
    RCCHECK()

    rc = rcl_init_options_fini(&init_options);

    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();

    rc = rcl_node_init(&node, "crazyflie_node", "", &context, &node_ops);
    RCCHECK()

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
    RCCHECK()

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
    RCCHECK()

    // Create publisher 3
    const char* drone_vel = "/cmd_vel";

    rcl_publisher_t pub_vel        = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * pub_type_support_vel= ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    rcl_publisher_options_t pub_opt_vel = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &pub_vel,
        &node,
        pub_type_support_vel,
        drone_vel,
        &pub_opt_vel);
    RCCHECK()


    // Create publisher 4
    const char* drone_tf = "/tf";

    rcl_publisher_t pub_tf  = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * pub_type_support_tf = ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage);
    rcl_publisher_options_t pub_opt_tf = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &pub_tf,
        &node,
        pub_type_support_tf,
        drone_tf,
        &pub_opt_tf);
    RCCHECK()

    // Init messages 
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

        rc = rcl_publish( &pub_attitude, (const void *) &pose, NULL);
        RCSOFTCHECK()

        rc = rcl_publish( &pub_odom, (const void *) &odom, NULL);
        RCSOFTCHECK()

        // geometry_msgs__msg__TransformStamped tf_msg;
        // tf_msg.header.frame_id.data = "/map";   
        // tf_msg.header.frame_id.size = strlen(tf_msg.header.frame_id.data);
        // tf_msg.header.frame_id.capacity = strlen(tf_msg.header.frame_id.data);

        // struct timespec ts;
        // clock_gettime(CLOCK_REALTIME, &ts);
        // tf_msg.header.stamp.sec =  ts.tv_sec;
        // tf_msg.header.stamp.nanosec = ts.tv_nsec;
        
        // tf_msg.child_frame_id.data = "/base_footprint_drone_attitude";
        // tf_msg.child_frame_id.size = strlen(tf_msg.child_frame_id.data);
        // tf_msg.child_frame_id.capacity = strlen(tf_msg.child_frame_id.data); 

        // quaternion q;
        // euler_to_quaternion(roll, pitch, yaw, &q);
        // tf_msg.transform.rotation.x = q.x;
        // tf_msg.transform.rotation.y = q.y;
        // tf_msg.transform.rotation.z = q.z;
        // tf_msg.transform.rotation.w = q.w;

        // tf_msg.transform.translation.x = x;
        // tf_msg.transform.translation.y = y;
        // tf_msg.transform.translation.z = z;

        // tf2_msgs__msg__TFMessage tf2_msg;
        // tf2_msg.transforms.size = 1;
        // tf2_msg.transforms.capacity = 1;
        // tf2_msg.transforms.data = &tf_msg;

        // rc = rcl_publish(&pub_tf, (const void *) &tf2_msg, NULL);
        // RCSOFTCHECK()

        geometry_msgs__msg__Twist msg;
        msg.angular.z = pose.y*3.14/180;
        msg.linear.x = pose.x*3.14/180;

        msg.angular.z = (fabs(msg.angular.z) > 0.70) ? sign(msg.angular.z)*0.70 : msg.angular.z;
        msg.linear.x = (fabs(msg.linear.x) > 0.20) ? sign(msg.linear.x)*0.20 : msg.linear.x;

        rc = rcl_publish( &pub_vel, (const void *) &msg, NULL);
        
        vTaskDelay(10/portTICK_RATE_MS);
    }
    
    vTaskSuspend( NULL );
}