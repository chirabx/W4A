#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Function declarations
void Move2goal(MoveBaseClient &ac, ros::Publisher &pub,double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw);
void SwingAndShoot(ros::Publisher &pub, double swing_speed, double swing_angle, int swing_times);
void sleep(double second)
{
    ros::Duration(second).sleep();
}

// Retry logic function
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw)
{
    ros::NodeHandle nh;
    geometry_msgs::Twist vel_msg;
    int count = 0;
    ros::Rate loop_rate(10);

    ROS_INFO("Executing backward retry logic...");
    vel_msg.linear.x = -0.1;
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
    Move2goal(ac, pub, x, y, yaw);
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

void SwingAndShoot(ros::Publisher &pub, double swing_speed, double swing_angle, int swing_times)
{
    if (swing_times < 1)
    {
        ROS_WARN("SwingAndShoot: swing_times=%d < 1, use 1 instead", swing_times);
        swing_times = 1;
    }
    if (swing_speed <= 0.0)
    {
        ROS_ERROR("SwingAndShoot: swing_speed=%.3f must be > 0, abort", swing_speed);
        return;
    }

    const double angle_first = swing_angle * M_PI / 180.0;   // 首摆角度(弧度)
    const double angle_full  = 2.0 * angle_first;            // 后续每摆角度
    const double pause_sec   = 0.20;
    ros::Rate loop_rate(50);
    geometry_msgs::Twist vel_msg;

    ROS_INFO("Laser ON, starting swing... speed=%.3f rad/s, angle=%.1f deg, times=%d",
             swing_speed, swing_angle, swing_times);

    for (int i = 0; i < swing_times && ros::ok(); i++)
    {
        const double target_rad = (i == 0) ? angle_first : angle_full;
        const int    dir        = (i % 2 == 0) ? 1 : -1;     // 偶数次向左,奇数次向右
        const double duration   = target_rad / swing_speed;
        const ros::Time t0      = ros::Time::now();

        vel_msg.angular.z = dir * swing_speed;
        ROS_INFO("  swing %d/%d : %s %.1f deg, %.2f s",
                 i + 1, swing_times, (dir > 0 ? "LEFT" : "RIGHT"),
                 target_rad * 180.0 / M_PI, duration);

        while (ros::ok() && (ros::Time::now() - t0).toSec() < duration)
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        vel_msg.angular.z = 0;
        pub.publish(vel_msg);
        ros::Duration(pause_sec).sleep();
    }
}

void Move2goal(MoveBaseClient &ac, ros ::Publisher &pub,double x, double y, double yaw)
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

    ac.cancelAllGoals();
    ros::Duration(0.15).sleep();

    ac.sendGoal(goal);
    ROS_INFO("MoveBase Send Goal !!!");
    ac.waitForResult();

    actionlib::SimpleClientGoalState state = ac.getState();

    switch (state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO("Target point (%.3f, %.3f, %.3f) reached successfully!", x, y, yaw);
        break;

    case actionlib::SimpleClientGoalState::ABORTED:
        ROS_WARN("Navigation aborted - possibly due to obstacles or path planning failure");
        performRetryLogic(ac, pub, x, y, yaw);
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
    // First target point
    Move2goal(ac, pub, 0.91, -0.90, -1.134);
    SwingAndShoot(pub, 0.20, 15.0, 1);  

    // Second target point
    Move2goal(ac, pub, 0.9, 1.53, 0.506);
    SwingAndShoot(pub, 0.20, 15.0, 1); 

    // Third target point
    Move2goal(ac, pub, 0.096, 1.506, 2.006);
    SwingAndShoot(pub, 0.20, 15.0, 1);

    // Fourth target point
    Move2goal(ac, pub, 0.11, 0.62, -2.321);
    SwingAndShoot(pub, 0.20, 15.0, 1);

    vel_msg.linear.x = -0.20;
    count = 0;
    while (ros::ok() && count < 25)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
        count++;
    }


    // Fifth target point
    Move2goal(ac, pub, 1.605, -0.843, -2.600);
    SwingAndShoot(pub, 0.20, 15.0, 1);

    // Sixth target point
    Move2goal(ac, pub, 2.394, -0.867, -1.184);
    SwingAndShoot(pub, 0.20, 15.0, 1);

    // Seventh target point
    Move2goal(ac, pub, 2.461, -0.2, 0.947);
    SwingAndShoot(pub, 0.20, 15.0, 1);

    vel_msg.linear.x = -0.20;
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


    // Eighth target point
    Move2goal(ac, pub, 1.65, 1.53, 2.006);
    SwingAndShoot(pub, 0.20, 15.0, 1);

    // Ninth target point
    Move2goal(ac, pub, 2.49, 1.38, 0.555);
    SwingAndShoot(pub, 0.20, 15.0, 3);


    // 【修改】完成所有动作，返回起始点后，关闭激光
    ros::service::waitForService("/close");
    shoot_close_client.call(empty_srv);
    ROS_INFO("Returned to start. Laser OFF.");
    return 0;
}