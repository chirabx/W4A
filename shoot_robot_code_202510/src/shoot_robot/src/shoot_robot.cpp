#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Function declarations
void Move_safe(ros::Publisher &pub, double linear_x, double linear_y, double distance);
void Move3goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name);
void Move2goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw, const std::string &tag_name);
void sleep(double second)
{
    ros::Duration(second).sleep();
}
void Move_safe(ros::Publisher &pub, double linear_x, double linear_y, double distance)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_x;
    vel_msg.linear.y = linear_y;
    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && count < distance)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    pub.publish(vel_msg);
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
    vel_msg.linear.x = -0.05;
    count = 0;
    while (ros::ok() && count < 10)
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

    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Target point %s (%.3f, %.3f, %.3f) reached successfully!", tag_name.c_str(), x, y, yaw);
        system(("roslaunch shoot_robot shoot_tag_" + tag_name + ".launch").c_str());
        break;

    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, x, y, yaw, tag_name);
        break;
    }
    // sleep(0.5);
}


void Turn_safe_1(ros::Publisher &pub, double angular_z, double distance)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = angular_z;
    int count = 0;
    ros::Rate loop_rate(10);
    while (ros::ok() && count < distance)
    {
        pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    // 停下
    vel_msg.angular.z = 0.0;
    pub.publish(vel_msg);
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

    int count = 0;
    ros::Rate loop_rate(10);
    shoot_close_client.call(empty_srv);

    // Move_safe(pub, 0.0, 0.4, 22);
    // Move_safe(pub, 0.4, 0.0, 30);
   

    Move2goal(ac, 0.91, -0.90, -1.134, "1");
    shoot_close_client.call(empty_srv);

    Move2goal(ac, 0.887, 1.53, 0.456, "3");
    shoot_close_client.call(empty_srv);

    Move2goal(ac, 0.076, 1.526, 2.006, "1");
    shoot_close_client.call(empty_srv);

    Move2goal(ac, 0.123, 0.820, -2.281, "2");
    shoot_close_client.call(empty_srv);

    Move_safe(pub,-0.4,0.0,10);

    Move2goal(ac, 1.632, -0.867, -2.700, "1");
    shoot_close_client.call(empty_srv);

    Move2goal(ac, 2.394, -0.867, -1.134, "1");
    shoot_close_client.call(empty_srv);

    Move2goal(ac, 2.471, -0.363, 0.947, "1");
    shoot_close_client.call(empty_srv);

    Move_safe(pub,-0.4,0.0,10);

    Move2goal(ac, 1.668, 1.509, 2.006, "1");
    //shoot_close_client.call(empty_srv);

    Move2goal(ac, 2.412, 1.344, 0.465, "3");
    //shoot_close_client.call(empty_srv);
    Turn_safe_1(pub, 0.1, 30);
    Turn_safe_1(pub, -0.1, 30);


    return 0;
}