#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 全局变量：ABORTED状态计数器
static int aborted_counter_tag2 = 0; // tag_name="2"的ABORTED计数
static int aborted_counter_tag3 = 0; // tag_name="3"的ABORTED计数
const int MAX_ABORTED_COUNT = 1;     // 最大ABORTED次数

// Function declarations
void Move2goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw, const std::string &tag_name);
void resetAbortedCounter(const std::string &tag_name);
bool shouldSkipDueToAbortedCount(const std::string &tag_name);
void sleep(double second)
{
    ros::Duration(second).sleep();
}

// 重置ABORTED计数器
void resetAbortedCounter(const std::string &tag_name)
{
    if (tag_name == "2")
    {
        aborted_counter_tag2 = 0;
        ROS_INFO("Reset ABORTED counter for tag_name=2");
    }
    else if (tag_name == "3")
    {
        aborted_counter_tag3 = 0;
        ROS_INFO("Reset ABORTED counter for tag_name=3");
    }
}

// 检查是否应该因为ABORTED计数而跳过
bool shouldSkipDueToAbortedCount(const std::string &tag_name)
{
    if (tag_name == "2")
    {
        if (aborted_counter_tag2 >= MAX_ABORTED_COUNT)
        {
            ROS_WARN("Tag_name=2 has reached maximum ABORTED count (%d), skipping navigation point", MAX_ABORTED_COUNT);
            return true;
        }
    }
    else if (tag_name == "3")
    {
        if (aborted_counter_tag3 >= MAX_ABORTED_COUNT)
        {
            ROS_WARN("Tag_name=3 has reached maximum ABORTED count (%d), skipping navigation point", MAX_ABORTED_COUNT);
            return true;
        }
    }
    return false;
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw, const std::string &tag_name)
{
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    int count = 0;
    ros::Rate loop_rate(10);

    ROS_INFO("Executing backward retry logic...");
    vel_msg.linear.x = -0.05; // Backward speed
    count = 0;
    while (ros::ok() && count < 10) // Backward 30 steps
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    ROS_INFO("Retrying to move to target point (%.3f, %.3f, %.3f)", x, y, yaw);
    Move2goal(ac, x, y, yaw, tag_name);
}

void Move2goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name)
{
    // 检查是否应该因为ABORTED计数而跳过
    if (shouldSkipDueToAbortedCount(tag_name))
    {
        return;
    }

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();

    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Current state: %s", state.toString().c_str());
    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Target point %s (%.3f, %.3f, %.3f) reached successfully!", tag_name.c_str(), x, y, yaw);
        // 成功到达后重置计数器
        resetAbortedCounter(tag_name);
        system(("roslaunch shoot_robot shoot_tag_" + tag_name + ".launch").c_str());
        break;

    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");

        // 对特定标签进行ABORTED计数
        if (tag_name == "2")
        {
            aborted_counter_tag2++;
            ROS_WARN("Tag_name=2 ABORTED count: %d/%d", aborted_counter_tag2, MAX_ABORTED_COUNT);
        }
        else if (tag_name == "3")
        {
            aborted_counter_tag3++;
            ROS_WARN("Tag_name=3 ABORTED count: %d/%d", aborted_counter_tag3, MAX_ABORTED_COUNT);
        }

        // 检查是否达到最大计数
        if (shouldSkipDueToAbortedCount(tag_name))
        {
            ROS_ERROR("Maximum ABORTED count reached for tag_name=%s, skipping this navigation point", tag_name.c_str());
            return;
        }
        performRetryLogic(ac, x, y, yaw, tag_name);
        break;
    }
    // sleep(0.5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shoot_robot_base");
    ros::NodeHandle nh;

    geometry_msgs::Twist vel_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceClient shoot_close_client;
    std_srvs::Empty empty_srv;

    shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    ros::Rate loop_rate(10);

    string input = "123";

    for (size_t i = 0; i < input.length(); ++i)
    {
        char ch = input[i];
        if (ch == '1')
        {
            // First target point
            Move2goal(ac, 0.840, -0.820, -0.785, "1");
        }
        else if (ch == '2')
        {
            // Second target point
            Move2goal(ac, 0.837, 1.463, 0.785, "1");
        }
        else if (ch == '3')
        {
            // Third target point
            Move2goal(ac, 0.156, 1.556, 2.355, "1");
        }
        else if (ch == '4')
        {
            // Fourth target point
            Move2goal(ac, 0.131, 0.799, -2.355, "1");
        }
        else if (ch == '5')
        {
            // Fifth target point
            Move2goal(ac, 2.364, -0.092, 0.785, "1");
        }
        else if (ch == '6')
        {
            // Sixth target point
            Move2goal(ac, 2.423, -0.837, -0.785, "1");
        }
        else if (ch == '7')
        {
            // Seventh target point
            Move2goal(ac, 1.662, -0.797, -2.355, "1");
        }
        else if (ch == '8')
        {
            // Eighth target point
            Move2goal(ac, 1.668, 1.489, 2.355, "1");
        }
        if (i != input.length()-1)
        {
            shoot_close_client.call(empty_srv);
        }
    }

    // Enemy base
    Move2goal(ac, 2.512, 1.404, 1.17, "2");
    shoot_close_client.call(empty_srv);

    // 重置tag计数器，为第二次尝试做准备
    resetAbortedCounter("3");
    resetAbortedCounter("2");
    // ###############################################################

    input = "8";

    for (size_t i = 0; i < input.length(); ++i)
    {
        char ch = input[i];
        if (ch == '1')
        {
            // First target point
            Move2goal(ac, 0.840, -0.820, -0.785, "1");
        }
        else if (ch == '2')
        {
            // Second target point
            Move2goal(ac, 0.837, 1.463, 0.785, "1");
        }
        else if (ch == '3')
        {
            // Third target point
            Move2goal(ac, 0.156, 1.556, 2.355, "1");
        }
        else if (ch == '4')
        {
            // Fourth target point
            Move2goal(ac, 0.131, 0.799, -2.355, "1");
        }
        else if (ch == '5')
        {
            // Fifth target point
            Move2goal(ac, 2.364, -0.092, 0.785, "1");
        }
        else if (ch == '6')
        {
            // Sixth target point
            Move2goal(ac, 2.423, -0.837, -0.785, "1");
        }
        else if (ch == '7')
        {
            // Seventh target point
            Move2goal(ac, 1.662, -0.797, -2.355, "1");
        }
        else if (ch == '8')
        {
            // Eighth target point
            Move2goal(ac, 1.668, 1.489, 2.355, "1");
        }
        if (i != input.length()-1)
        {
            shoot_close_client.call(empty_srv);
        }
    }

    // Enemy base
    Move2goal(ac, 2.512, 1.404, 1.17, "2");
    shoot_close_client.call(empty_srv);

    return 0;
}
