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
void sleep(double second)
{
    ros::Duration(second).sleep();
}

void Move2goal(MoveBaseClient& ac, double x, double y, double yaw)
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

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
        system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
        ros::NodeHandle nh;

        geometry_msgs::Twist vel_msg;
        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        int count = 0;
        ros::Rate loop_rate(10);
        vel_msg.linear.x = -0.05;//cbx
        count = 0;
        while (ros::ok() && count < 30) // yyx
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);

        Move2goal(ac, x, y, yaw);

    }
    // sleep(0.5);
}

void Move1goal(MoveBaseClient& ac, double x, double y, double yaw)
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

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The Goal 1 Reached Successfully!!!");
        // system("roslaunch shoot_robot shoot_tag_1.launch");
    }
    else
    {
        ROS_WARN("The Goal Planning Failed for some reason");
        ros::NodeHandle nh;

        geometry_msgs::Twist vel_msg;
        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        int count = 0;
        ros::Rate loop_rate(10);
        vel_msg.linear.x = -0.05;//cbx
        count = 0;
        while (ros::ok() && count < 30) // yyx
        {
            pub.publish(vel_msg);
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
        // 停下
        vel_msg.linear.x = 0.0;
        pub.publish(vel_msg);

        Move1goal(ac, x, y, yaw);
    }
        
}

int main(int argc, char** argv)
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


    //xia 45

    //第一个目标点
    Move2goal(ac, 0.893, -0.748, -0.999);
    shoot_close_client.call(empty_srv);

    // //第二个目标点
    Move2goal(ac, 0.843, 1.583, 0.698);   
    shoot_close_client.call(empty_srv);

    // //第三个目标点
    Move2goal(ac, 0.178, 1.643, 2.540); 
    shoot_close_client.call(empty_srv);

    //第四个目标点
    Move2goal(ac, 0.140, 0.806, -2.301);
    shoot_close_client.call(empty_srv);

    //第五个目标点
    Move2goal(ac, 2.374, -0.062, 0.84);  
    shoot_close_client.call(empty_srv);

    //第六个目标点
    Move2goal(ac, 2.413, -0.767, -0.866);   
    shoot_close_client.call(empty_srv);

    //第七个目标点
    Move2goal(ac, 1.702, -0.837, -2.424); 
    shoot_close_client.call(empty_srv);

    //第八个目标点
    Move2goal(ac, 1.648, 1.509, 2.346);
    shoot_close_client.call(empty_srv);

    //第九个目标点
    Move2goal(ac, 2.392, 1.514, 0.808);
    shoot_close_client.call(empty_srv);

    return 0;

}
