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

void SwingAndShoot(ros ::Publisher &pub)
{
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(10);
    ROS_INFO("Laser ON, starting swing...");
    // 参数
    const double swing_speed = 0.27;      // 角速度 rad/s 0.27
    const double swing_angle = 0.262;   // 15度 = π/12 弧度
    const int one_way_steps = (int)(swing_angle / swing_speed / 0.1);  // 约10步
    // 左摆30度
    vel_msg.angular.z = swing_speed;
    for (int i = 0; i < one_way_steps && ros::ok(); i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    // 右摆60度（从左20度 → 右20度）
    vel_msg.angular.z = -swing_speed;
    for (int i = 0; i < one_way_steps * 2 && ros::ok(); i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    // // 回正30度（从右20度 → 中心）
    // vel_msg.angular.z = swing_speed;
    // for (int i = 0; i < one_way_steps && ros::ok(); i++)
    // {
    //     pub.publish(vel_msg);
    //     loop_rate.sleep();
    // }
    //停止
    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
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
    Move2goal(ac, pub,2.42, 0.78, 0.785, "1");
    
    // //Second target point H
    Move2goal(ac, pub,2.34, 0.05, -0.785, "1");

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
    Move2goal(ac, pub,1.67, 0.05, -2.355, "1");
    
    // Fourth target point
    Move2goal(ac, pub,1.55, 2.40, 2.355, "1");
    
    // Fifth target point
    Move2goal(ac, pub,2.43, 2.36, 0.785, "1");//(2.5,2.41,0.785)
    
    // Sixth target point
    Move2goal(ac, pub,2.35, 1.53, -0.785, "1");
    
    Move1goal(ac,1.40,1.40,-3.14);
    sleep(0.5);
    // Seventh target point
    Move2goal(ac, pub,0.08, 1.72, -2.355, "1");

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
