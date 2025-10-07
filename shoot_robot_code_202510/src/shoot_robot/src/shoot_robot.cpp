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
void Move2goal(MoveBaseClient &ac, double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, double x, double y, double yaw, const std::string &tag_name);
void sleep(double second)
{
    ros::Duration(second).sleep();
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
    while (ros::ok() && count < 30) // Backward 30 steps
    {
        pub.publish(vel_msg);
        ros::spinOnce();
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

    case actionlib::SimpleClientGoalState::PREEMPTED:
        ROS_WARN("Navigation preempted - possibly due to new target point");
        performRetryLogic(ac, x, y, yaw, tag_name);
        break;

    case actionlib::SimpleClientGoalState::REJECTED:
        ROS_ERROR("Target point rejected - possibly invalid target point %s (%.3f, %.3f, %.3f)", tag_name.c_str(), x, y, yaw);
        ROS_WARN("Skipping current target point, continuing to next target");
        return;

    case actionlib::SimpleClientGoalState::RECALLED:
        ROS_WARN("Target cancelled");
        performRetryLogic(ac, x, y, yaw, tag_name);
        break;

    case actionlib::SimpleClientGoalState::LOST:
        ROS_ERROR("Communication lost with move_base server");
        ROS_ERROR("Program will exit, please check move_base node status");
        return;

    default:
        ROS_ERROR("Unknown navigation state: %s", state.toString().c_str());
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

    int count = 0;
    ros::Rate loop_rate(10);


    // First target point
    Move2goal(ac, 0.810, -0.890, -0.785, "1");
    shoot_close_client.call(empty_srv);

    // //Second target point
    Move2goal(ac, 0.847, 1.483, 0.785, "1");
    shoot_close_client.call(empty_srv);

    // //Third target point
    Move2goal(ac, 0.116, 1.516, 2.355, "1");
    shoot_close_client.call(empty_srv);

    // Fourth target point
    Move2goal(ac, 0.131, 0.799, -2.355, "1");
    shoot_close_client.call(empty_srv);

    // Fifth target point
    Move2goal(ac, 2.354, -0.062, 0.785, "1");
    shoot_close_client.call(empty_srv);

    // Sixth target point
    Move2goal(ac, 2.433, -0.767, -0.785, "1");
    shoot_close_client.call(empty_srv);

    // Seventh target point
    Move2goal(ac, 1.642, -0.817, -2.355, "1");
    shoot_close_client.call(empty_srv);

    // Eighth target point
    Move2goal(ac, 1.668, 1.529, 2.355, "1");
    shoot_close_client.call(empty_srv);

    // Enemy base
    Move2goal(ac, 2.392, 1.534, 0.785, "2");
    // Move2goal(ac, 2.392, 1.534, 0.785, "3");
    shoot_close_client.call(empty_srv);

    return 0;
}
