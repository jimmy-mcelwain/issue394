#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <rcl/rcl.h>
#include <rcl/types.h>
#include <rcl/publisher.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rmw/qos_profiles.h>
#include <rmw_microros/ping.h>

#include <sensor_msgs/msg/joint_state.h>

#define JOINT_COUNT 15

rcl_timer_t Ros_StateServer_Timer;
rclc_executor_t Ros_StateServer_Executor;

rcl_publisher_t publisherJointState;
sensor_msgs__msg__JointState *msgJointState;

typedef int BOOL;

#define MAX_JOINT_NAME_LENGTH 32

void Ros_StateServer_TimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time; // unused

    if (timer != NULL)
    {
        BOOL bRet;
        struct timespec timestamp;

        // timestamp
        clock_gettime(0, &timestamp);

        msgJointState->header.stamp.sec = timestamp.tv_sec;
        msgJointState->header.stamp.nanosec = timestamp.tv_nsec;

        for (int i = 0; i < JOINT_COUNT; i += 1)
        {
            msgJointState->position.data[i] = i;
            msgJointState->velocity.data[i] = i + 1;
            msgJointState->effort.data[i] = i + 2;
            msgJointState->name.size = i + 1;
            msgJointState->position.size = i + 1;
            msgJointState->velocity.size = i + 1;
            msgJointState->effort.size = i + 1;
            rcl_ret_t ret = rcl_publish(&publisherJointState, msgJointState, NULL);
            RCL_UNUSED(ret);
            usleep(20);
        }
    }
}

int main()
{
    rcl_allocator_t rclAllocator;
    rclc_support_t rclSupport;
    rcl_node_t rclNode;
    rcl_ret_t ret;

    // Wait for agent to become available
    do
    {
        usleep(1000);
        ret = rmw_uros_ping_agent(1000, 2);
    } while (ret != RCL_RET_OK);

    //=============================================
    rclAllocator = rcl_get_default_allocator();

    rclc_support_init(&rclSupport, 0, NULL, &rclAllocator);
    rclc_node_init_default(&rclNode, "testnode", "", &rclSupport);

    //=============================================

    printf("%s, %p, %p \n", ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState)->typesupport_identifier, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState)->data, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState)->func);

    rmw_qos_profile_t const *const qos_profile = &rmw_qos_profile_sensor_data;

    rclc_publisher_init(
        &publisherJointState,
        &rclNode,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states",
        qos_profile);

    msgJointState = sensor_msgs__msg__JointState__create();
    rosidl_runtime_c__String__Sequence__init(&msgJointState->name, JOINT_COUNT);
    rosidl_runtime_c__float64__Sequence__init(&msgJointState->position, JOINT_COUNT);
    rosidl_runtime_c__float64__Sequence__init(&msgJointState->velocity, JOINT_COUNT);
    rosidl_runtime_c__float64__Sequence__init(&msgJointState->effort, JOINT_COUNT);

    rosidl_runtime_c__String__assign(&msgJointState->header.frame_id, "");

    char formatBuffer[MAX_JOINT_NAME_LENGTH];
    int robotIterator = 0;
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        snprintf(formatBuffer, MAX_JOINT_NAME_LENGTH, "group_1/joint_%d", i);
        rosidl_runtime_c__String__assign(&msgJointState->name.data[i], formatBuffer);
    }

    //=============================================
    rclc_timer_init_default(
        &Ros_StateServer_Timer,
        &rclSupport,
        RCL_MS_TO_NS(20),
        Ros_StateServer_TimerCallback);

    Ros_StateServer_Executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&Ros_StateServer_Executor, &rclSupport.context, 1, &rclAllocator);
    rclc_executor_add_timer(&Ros_StateServer_Executor, &Ros_StateServer_Timer);

    rclc_executor_spin(&Ros_StateServer_Executor);
}
