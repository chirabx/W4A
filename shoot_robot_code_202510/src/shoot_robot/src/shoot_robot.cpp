#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");

    MoveBaseClient ac("move_base", true);

    ac.waitForServer();

    // 选择离自己最近的 4 个普通靶标位置
    move_base_msgs::MoveBaseGoal goal1;

    // 对方基地点位
    move_base_msgs::MoveBaseGoal goal2;

    // 返回点位
    move_base_msgs::MoveBaseGoal goal3;

    // 第一个普通靶标在 map 坐标系下的坐标位置，前方0.5米，向右90度
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, -1.5707);
    goal1.target_pose.pose.position.x = 0.5;
    goal1.target_pose.pose.position.y = 0.0;
    goal1.target_pose.pose.orientation.z = quaternion.z();
    goal1.target_pose.pose.orientation.w = quaternion.w();
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal1);
    ROS_INFO("Send Goal  1 !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 第二个普通靶标在 map 坐标系下的坐标位置，前方1.0米，向右90度
    goal1.target_pose.pose.position.x = 1.0;
    goal1.target_pose.pose.position.y = 0.0;
    goal1.target_pose.pose.orientation.z = quaternion.z();
    goal1.target_pose.pose.orientation.w = quaternion.w();
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal1);
    ROS_INFO("Send Goal 2 !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 2 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 第三个普通靶标在 map 坐标系下的坐标位置，前方1.0米，面向正前方
    quaternion.setRPY(0, 0, 0);
    goal1.target_pose.pose.position.x = 1.0;
    goal1.target_pose.pose.position.y = 0.0;
    goal1.target_pose.pose.orientation.z = quaternion.z();
    goal1.target_pose.pose.orientation.w = quaternion.w();
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal1);
    ROS_INFO("Send Goal 3 !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 3 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }


    // 第四个普通靶标在 map 坐标系下的坐标位置，前方0.5米，面向左侧
    quaternion.setRPY(0, 0, 1.5707);
    goal1.target_pose.pose.position.x = 0.5;
    goal1.target_pose.pose.position.y = 0.0;
    goal1.target_pose.pose.orientation.z = quaternion.z();
    goal1.target_pose.pose.orientation.w = quaternion.w();
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal1);
    ROS_INFO("Send Goal 4 !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 4 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 对方基地在 map 坐标系下的坐标位置,前方 1 米，右侧2米，朝向右侧
    quaternion.setRPY(0, 0, -1.5707);
    goal2.target_pose.pose.position.x = 1.0;
    goal2.target_pose.pose.position.y = -2.0;
    goal2.target_pose.pose.orientation.z = quaternion.z();
    goal2.target_pose.pose.orientation.w = quaternion.w();
    goal2.target_pose.header.frame_id = "map";
    goal2.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal2);
    ROS_INFO("Send Goal Enemy Target !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal Enemy Target Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_2.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    // 返回出发点
    goal3.target_pose.pose.position.x = 0.0;
    goal3.target_pose.pose.position.y = 0.0;
    goal3.target_pose.pose.orientation.z = 0.0;
    goal3.target_pose.pose.orientation.w = 1.0;
    goal3.target_pose.header.frame_id = "map";
    goal3.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal3);
    ROS_INFO("Send Goal Home !!!");
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Back !!!!");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
    }

    return 0;
}
