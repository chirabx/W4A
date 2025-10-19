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
static int aborted_counter_tag1 = 0; // tag_name="1"的ABORTED计数
const int MAX_ABORTED_COUNT = 100;     // 最大ABORTED次数
const int MAX_ABORTED_COUNT1 = 100;     // 最大ABORTED次数

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
    else if (tag_name == "1")
    {
        aborted_counter_tag1 = 0;
        ROS_INFO("Reset ABORTED counter for tag_name=1");
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
    else if (tag_name == "1")
    {
        if (aborted_counter_tag1 >= 2)
        {
            ROS_WARN("Tag_name=1 has reached maximum ABORTED count (%d), skipping navigation point", MAX_ABORTED_COUNT1);
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
    Move1goal(ac, x, y, yaw);
}

void Move1goal(MoveBaseClient &ac, double x, double y, double yaw)
{
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
    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, x, y, yaw, "1");
        break;
    }
    // sleep(0.5);
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
        else if (tag_name == "1")
        {
            aborted_counter_tag1++;
            ROS_WARN("Tag_name=1 ABORTED count: %d/%d", aborted_counter_tag1, MAX_ABORTED_COUNT);
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
    ros::NodeHandle nh_;

    geometry_msgs::Twist vel_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceClient shoot_close_client;
    ros::ServiceClient shoot_client;
    std_srvs::Empty empty_srv;

    shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();
    ros::Rate loop_rate(10);
    int count = 0;

    shoot_client.call(empty_srv);
    vel_msg.linear.y = 0.20; // Backward speed
    count = 0;
    loop_rate.sleep();
    while (ros::ok() && count < 10) // Backward 30 steps
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);

    sleep(0.5);

   vel_msg.linear.x = 1.20; // Backward speed
    count = 0;
    while (ros::ok() && count < 60) // Backward 30 steps
    {
        ROS_INFO("go");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    // Eighth target point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Move1goal(ac, 1.688, 1.449, 2.355);

    //向右
    vel_msg.angular.z = -0.1;
    count = 0;
    while (ros::ok() && count < 15)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }

    Move1goal(ac, 2.2, 0.9, 0.5);
    // Enemy base!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Move1goal(ac, 2.482, 1.044, 1.37);

    sleep(0.5);

    vel_msg.linear.y = -0.15; // Backward speed
    count = 0;
    while (ros::ok() && count < 20) // Backward 30 steps
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);

    sleep(0.5);

    vel_msg.linear.x = 0.10; // Backward speed
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

    //向右
    vel_msg.angular.z = -0.15;
    count = 0;
    while (ros::ok() && count < 15)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.15;
    count = 0;
    while (ros::ok() && count < 30)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //向右
    vel_msg.angular.z = -0.15;
    count = 0;
    while (ros::ok() && count < 45)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.15;
    count = 0;
    while (ros::ok() && count < 60)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向右
    vel_msg.angular.z = -0.15;
    count = 0;
    while (ros::ok() && count < 60)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.15;
    count = 0;
    while (ros::ok() && count < 30)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);

    // 后撤准备补
    vel_msg.linear.x = -0.15;
    count = 0;
    while (ros::ok() && count < 10)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);
    // 左移准备补
    vel_msg.linear.y = 0.15;
    count = 0;
    while (ros::ok() && count < 10)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);
    //射击准备
    Move1goal(ac, 2.2, 0.9, 0.5);
    
    // Fifth target point#####################################################
    Move1goal(ac, 2.244, -0.138, 0.785);

    //向右
    vel_msg.angular.z = -0.1;
    count = 0;
    while (ros::ok() && count < 15)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.1;
    count = 0;
    while (ros::ok() && count < 30)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }

   vel_msg.linear.x = -0.50; // Backward speed
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

    sleep(0.5);

   vel_msg.linear.y = 0.40; // Backward speed
    count = 0;
    while (ros::ok() && count < 10) // Backward 30 steps
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);

    Move1goal(ac, 2.2, 0.9, 0.5);
    // Enemy base#####################################################
    Move1goal(ac, 2.482, 1.044, 1.37);

    sleep(0.5);

    vel_msg.linear.y = -0.15; // Backward speed
    count = 0;
    while (ros::ok() && count < 20) // Backward 30 steps
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);

    sleep(0.5);

    vel_msg.linear.x = 0.10; // Backward speed
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

    //向右
    vel_msg.angular.z = -0.15;
    count = 0;
    while (ros::ok() && count < 15)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.15;
    count = 0;
    while (ros::ok() && count < 30)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);

//#####################################################
    //向右
    vel_msg.angular.z = -0.15;
    count = 0;
    while (ros::ok() && count < 45)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.15;
    count = 0;
    while (ros::ok() && count < 60)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向右
    vel_msg.angular.z = -0.15;
    count = 0;
    while (ros::ok() && count < 60)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    //向左
    vel_msg.angular.z = 0.15;
    count = 0;
    while (ros::ok() && count < 60)
    {
        ROS_INFO("shoot");
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);
}
