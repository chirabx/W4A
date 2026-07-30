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
<<<<<<< HEAD
=======

>>>>>>> 改了导航点，加后退，改了出发方式

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
    Move_safe(pub, 0.0, 0.30, 25);
    Move_safe(pub, 0.30, 0.0, 27);

    // First target point
<<<<<<< HEAD
    
    
    
    Move2goal(ac, 2.308, 0.628, 0.785, "1");  //G Move2goal(ac, 2.358, 0.668, 0.761, "1"); 
    shoot_close_client.call(empty_srv);

    // vel_msg.linear.x 




    // count = 0;
    // while (ros::ok() && count < 10)
    // {
    //    pub.publish(vel_msg);
    //    loop_rate.sleep();
    //    count++;
    // }
    

    // //Second target point
    
    Move2goal(ac, 2.284, -0.09, -0.785, "1");  //H
    shoot_close_client.call(empty_srv);

   

    //vel_msg.linear.x = -0.05;
    //count = 0;
    //while (ros::ok() && count < 20)
    //{
      //  pub.publish(vel_msg);
      //  loop_rate.sleep();
      //  count++;
    //}
    // Stop
    //vel_msg.linear.x = 0.0;
    //pub.publish(vel_msg);

    // //Third target point
   
    
=======
    Move2goal(ac, 2.308, 0.628, 0.785, "1");  //G
    shoot_close_client.call(empty_srv);


    // //Second target point
    Move2goal(ac, 2.284, -0.09, -0.785, "1");  //H
    shoot_close_client.call(empty_srv);

    
   

    // //Third target point0
>>>>>>> 改了导航点，加后退，改了出发方式
    Move2goal(ac, 1.538, -0.009, -2.355, "1");  // I
    shoot_close_client.call(empty_srv);

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 10)
    // {
    //    pub.publish(vel_msg);
    //    loop_rate.sleep();
    //    count++;
    // }

    // Fourth target point
    Move2goal(ac, 1.703, 2.36, 2.355, "1");  //D
    shoot_close_client.call(empty_srv);
    //Move2goal(ac, 1.633, 2.316, 2.322, "1");  //D
    //shoot_close_client.call(empty_srv);

<<<<<<< HEAD
    
    //Move1goal(ac, 1.100, 0.400, 0);

    // Fifth target point
    Move2goal(ac, 2.356, 2.176, 0.785, "1");  //E Move2goal(ac, 2.456, 2.276, 0.48, "1");  
    shoot_close_client.call(empty_srv);

    
    

    // // Sixth target point
    //Move2goal(ac, 2.379, 1.443, -0.876, "1");//F
    //shoot_close_client.call(empty_srv);
    
    // Seventh target point
    Move2goal(ac, 2.279, 1.543, -0.785, "1");//F
=======

    // Fifth target point
    Move2goal(ac, 2.306, 2.106, 0.785, "1");  //E
    shoot_close_client.call(empty_srv);

    

    // // Sixth target point
    Move2goal(ac, 2.279, 1.543, -0.785, "1");//F
    shoot_close_client.call(empty_srv);
    // Seventh target point
    Move2goal(ac, 0.083, 1.707, 3.92, "1");  //A
>>>>>>> 改了导航点，加后退，改了出发方式
    shoot_close_client.call(empty_srv);
    //Move2goal(ac, 1.633, 2.316, 2.322, "1");  //D
    //shoot_close_client.call(empty_srv);

    
<<<<<<< HEAD
    Move2goal(ac, 0.053, 1.647, 2.355, "1");  //A
    shoot_close_client.call(empty_srv);
   // Move1goal(ac, 0.877, 0.3, 1.57);
    

    
    
    //Move2goal(ac, 2.418, 0.728, 0.761, "1");  //G Move2goal(ac, 2.358, 0.668, 0.761, "1"); 
    //shoot_close_client.call(empty_srv);

    

    //vel_msg.linear.x = -0.05;
    //count = 0;
    //while (ros::ok() && count < 30)
   // {
    //    pub.publish(vel_msg);
    //    loop_rate.sleep();
    //    count++;
    //}
    // Stop1z
    //vel_msg.linear.x = 0.0;
   // pub.publish(vel_msg);

    // Eighth target point
    
    Move2goal(ac, 0.068, 2.374, 2.355, "1");  //B
    shoot_close_client.call(empty_srv);
    
    // Enemy base
    // Move2goal(ac, 2.412, 1.544, 0.785, "3");
    //Move2goal(ac, 1.538, -0.009, -2.286, "1");  // I
    //shoot_close_client.call(empty_srv);
    Move2goal(ac, 0.855, 2.258, 0.83, "1");  //C Move2goal(ac, 0.899, 2.308, 0.83, "1");  
    shoot_close_client.call(empty_srv);
=======

    // Eighth target point
    Move2goal(ac, 0.075, 2.364, 2.355, "1");  //B
    shoot_close_client.call(empty_srv);
    Move_safe(pub, -0.2, 0.0, 8);
    //Move1goal(ac, 0.5, 2.2, 0); 
    // Enemy base
    // Move2goal(ac, 2.412, 1.544, 0.785, "3");
    Move2goal(ac, 0.66, 2.1, 0.83, "1");  //C 
    shoot_close_client.call(empty_srv);

    Move1goal(ac, 0.03, 0.03, 0.0);
    Move_safe(pub, 0.0, -0.2, 15);
    //Move_safe(pub, -0.2, 0.0, 12);
>>>>>>> 改了导航点，加后退，改了出发方式

    // vel_msg.linear.x = -0.05;
    // count = 0;
    // while (ros::ok() && count < 10)
    // {
    //    pub.publish(vel_msg);
    //    loop_rate.sleep();
    //    count++;
    // }
    //返回出发点
    //Move1goal(ac, 1.55, 0.500, -1.7);
    //Move2goal(ac, 0.05, 0.15, 0,"1");
    //Move_safe(pub, 0.0, -0.1, 25); //30
    //Move_safe(pub, -0.1, 0.0, 15); //30
    Move1goal(ac, 0.7, 1.0, 0.0);
    Move_safe(pub, 0.0, -0.1, 100);
    Move_safe(pub, -0.1, 0.0, 70);
    
    return 0;
}