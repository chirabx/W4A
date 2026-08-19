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
    const double swing_speed = 0.15;      // 角速度 rad/s
    const double swing_angle = 0.175;   // 20度 = π/6 弧度
    const int one_way_steps = (int)(swing_angle / swing_speed / 0.1);  // 约10步
    // 左摆20度
    vel_msg.angular.z = swing_speed;
    for (int i = 0; i < one_way_steps && ros::ok(); i++)
    {
        pub.publish(vel_msg);
        loop_rate.sleep();
    }
    // 右摆40度（从左20度 → 右20度）
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
    // 同时声明开启和关闭激光的服务客户端
    ros::ServiceClient shoot_close_client = nh.serviceClient<std_srvs::Empty>("/close");
    ros::ServiceClient shoot_open_client = nh.serviceClient<std_srvs::Empty>("/shoot");
    std_srvs::Empty empty_srv;
    MoveBaseClient ac("move_base", true);
    ac.waitForServer();

    int count = 0;
    ros::Rate loop_rate(10);
    // 程序开始时，常开激光
    ros::service::waitForService("/shoot");
    shoot_open_client.call(empty_srv);
    ROS_INFO("Laser ON (Always on until return)");
    
    //Move_safe(pub,0.0,0.4,25);
    //Move_safe(pub,0.4,0.0,25);
   // Move1goal(ac, 0.78, 1.185, 0);
    sleep(0.5);

    // First target point G
    Move2goal(ac, pub,2.43, 0.79, 0.785, "1");
    // shoot_close_client.call(empty_srv);

    //Move1goal(ac, 0.877, 0.3, 1.57);

    // //Second target point H
    Move2goal(ac, pub,2.37, 0.01, -0.82, "1");
    // shoot_close_client.call(empty_srv);

    // //Third target point I
    Move2goal(ac, pub,1.58, 0.03, -2.420, "1");
    // shoot_close_client.call(empty_srv);

    // Fourth target point
    Move2goal(ac, pub,1.56, 2.33, 2.2, "1");
    // shoot_close_client.call(empty_srv);
    //Move_safe(pub,-0.4,0.0,10);
    // Move1goal(ac, 1.100, 0.400, 0);

    // Fifth target point
    Move2goal(ac, pub,2.44, 2.37, 0.95, "1");//(2.5,2.41,0.785)
    // shoot_close_client.call(empty_srv);

    // Sixth target point
    Move2goal(ac, pub,2.39, 1.52, -0.835, "1");
    // shoot_close_client.call(empty_srv);

    //Move1goal(ac,1.72,1.21,-1.57);两箱子之间

    // Seventh target point
    Move2goal(ac, pub,0.04, 1.62, -2.365, "1");
    // shoot_close_client.call(empty_srv);

    // Eighth target point
    Move2goal(ac, pub,0.08, 2.40, 2.355, "1");
    // shoot_close_client.call(empty_srv);

    // nineth target point
    Move2goal(ac, pub,0.87, 2.34, 0.799, "1");
    // shoot_close_client.call(empty_srv);

    // Move1goal(ac, 0.55, 0.75, 0);
    // sleep(0.5);
    Move1goal(ac, 0.1, 0.1, -1.57);//(0.05,0.05,0)
    Move_safe(pub,0.0,-0.4,5);
    Move_safe(pub,0.4,0.0,15);
    
    
    // 完成所有动作，返回起始点后，关闭激光
    ros::service::waitForService("/close");
    shoot_close_client.call(empty_srv);
    ROS_INFO("Returned to start. Laser OFF.");
    return 0;
}
