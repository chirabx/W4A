#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 射击点结构体
struct ShootPoint
{
    double x, y, yaw;
    std::string description;
    std::string launch_file;
};

// 导航到指定点并执行射击的辅助函数
bool navigateAndShoot(MoveBaseClient &ac, const ShootPoint &point, int goal_number)
{
    // 设置目标点
    move_base_msgs::MoveBaseGoal goal;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, point.yaw);

    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // 发送目标并等待结果
    ac.sendGoal(goal);
    ROS_INFO("Send Goal %d: %s", goal_number, point.description.c_str());
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal %d reached successfully!", goal_number);
        if (!point.launch_file.empty())
        {
            std::string command = "roslaunch shoot_robot " + point.launch_file;
            system(command.c_str());
        }
        return true;
    }
    else
    {
        ROS_WARN("Goal %d planning failed for some reason", goal_number);
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    // 定义所有射击点
    std::vector<ShootPoint> shoot_points = {
        // 普通靶标射击点
        {0.5, 0.0, -1.5707, "第1个普通靶标(右转90度)", "shoot_tag_1.launch"},
        {1.0, 0.0, -1.5707, "第2个普通靶标(右转90度)", "shoot_tag_1.launch"},
        {1.0, 0.0, 0.0, "第3个普通靶标(正前方)", "shoot_tag_1.launch"},
        {0.5, 0.0, 1.5707, "第4个普通靶标(左转90度)", "shoot_tag_1.launch"},
        // 敌方基地射击点
        {1.0, -2.0, -1.5707, "敌方基地(右转90度)", "shoot_tag_2.launch"},
        // 返回起始点
        {0.0, 0.0, 0.0, "返回起始点", ""}};

    // 依次执行所有射击点
    for (size_t i = 0; i < shoot_points.size(); ++i)
    {
        navigateAndShoot(ac, shoot_points[i], i + 1);
    }

    ROS_INFO("All shooting tasks completed!");
    return 0;
}