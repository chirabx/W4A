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
void Move2goal(MoveBaseClient &ac, ros::Publisher &pub,double x, double y, double yaw, string tag_name);
void Move1goal(MoveBaseClient &ac, double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, const std::string &tag_name);
void sleep(double second)
{
    ros::Duration(second).sleep();
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw, const std::string &tag_name)
{
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;
    int count = 0;
    ros::Rate loop_rate(10);

    ROS_INFO("Executing backward retry logic...");
    vel_msg.linear.x = -0.2;//0.05
    count = 0;
    while (ros::ok() && count < 15)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }
    // Stop
    vel_msg.linear.x = 0.0;
    pub.publish(vel_msg);

    ROS_INFO("Retrying to move to target point (%.3f, %.3f, %.3f)", x, y, yaw);
    Move2goal(ac, pub, x, y, yaw, tag_name);
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
// ================================================================
// 分段摆动扫射（改造版）：转5° → 停0.5s → 回摆5° → 停0.5s → 离开
// 纯导航方案：不依赖摄像头，利用 Turn_safe_1 固定时长旋转实现
// 命中原理：旋转段光斑在靶面移动，扫过 AprilTag 白区/黑框产生亮暗脉冲
// 激光：调用前由主程序开启（常亮贯穿），本函数不碰激光，
//       结束后由主程序在去下一航点前统一关闭
// ================================================================
void SwingAndShoot(ros::Publisher &pub)
{
    ROS_INFO("SwingAndShoot: left 5deg x2, right 5deg x4");

    // 参数（10Hz 步进，每步 0.1s）
    const double swing_speed = 0.22;   // 0.22 × 0.4s = 0.088rad ≈ 5.04°
    const int    swing_steps = 4;      // 旋转步数 = 0.4s（≈5°）
    const int    pause_steps = 8;      // 停顿步数 = 0.5s（若不要停顿，删掉这两行即可）

    // 左转 5° × 2
    for (int i = 0; i < 1; i++)
    {
        Turn_safe_1(pub,  swing_speed, swing_steps);
        Turn_safe_1(pub,  0.0, pause_steps);
    }

    // 右转 5° × 4（负号 = 右转）
    for (int i = 0; i < 2; i++)
    {
        Turn_safe_1(pub, -swing_speed, swing_steps);
        Turn_safe_1(pub,  0.0, pause_steps);
    }

    ROS_INFO("Swing done, moving to next waypoint.");
}



void Move2goal(MoveBaseClient &ac, ros ::Publisher &pub,double x, double y, double yaw, string tag_name)
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
        SwingAndShoot(pub);
        break;

    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, pub, x, y, yaw, tag_name);
        break;
    }
    // sleep(0.5);
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
    // 【修改】同时声明开启和关闭激光的服务客户端
    ros::ServiceClient shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    ros::ServiceClient shoot_open_client = nh.serviceClient<std_srvs::Empty>("/shoot");
    std_srvs::Empty empty_srv;
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    int count = 0;
    ros::Rate loop_rate(10);
    // 【修改】程序开始时，常开激光
    ros::service::waitForService("/shoot");
    shoot_open_client.call(empty_srv);
    ROS_INFO("Laser ON (Always on until return)");
    
    // Move_safe(pub,0.0,0.4,30);
    // Move_safe(pub,0.4,0.0,25);
    // Move_safe(pub,0.0,0.4,20);
    // sleep(0.5);

    // First target point G
    Move2goal(ac, pub,2.44, 0.77, 0.785, "1");
    
    // //Second target point H
    Move2goal(ac, pub,2.34, 0.01, -0.785, "1");

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 20)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    //     count++;
    // }
    // // Stop
    // vel_msg.linear.x = 0.0;
    // pub.publish(vel_msg);

    // //Third target point I
    Move2goal(ac, pub,1.63, 0.07, -2.355, "1");
    
    // Fourth target point
    Move2goal(ac, pub,1.59, 2.42, 2.355, "1");
    
    // Fifth target point
    Move2goal(ac, pub,2.41, 2.36, 0.785, "1");//(2.5,2.41,0.785)
    
    // Sixth target point
    Move2goal(ac, pub,2.35, 1.51, -0.785, "1");
    
    Move1goal(ac,1.40,1.40,-3.14);
    sleep(0.5);
    // Seventh target point
    Move2goal(ac, pub,0.08, 1.74, -2.355, "1");

    // Eighth target point
    Move2goal(ac, pub,0.06, 2.43, 2.355, "1");//x0.12 y2.50
    
    // nineth target point
    Move2goal(ac, pub,0.91, 2.38, 0.785, "1");
    
    Move1goal(ac, 0.05, 0.05, 0);//(0.05,0.05,0)
    // Move_safe(pub,0.0,-0.4,15);
    // Move_safe(pub,-0.4,0.0,15);
    // 【修改】完成所有动作，返回起始点后，关闭激光
    ros::service::waitForService("/close");
    shoot_close_client.call(empty_srv);
    ROS_INFO("Returned to start. Laser OFF.");
    return 0;
}
