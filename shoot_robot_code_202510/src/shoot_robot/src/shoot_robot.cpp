#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Function declarations
bool Move2goal(MoveBaseClient &ac, ros::Publisher &pub,double x, double y, double yaw);
void performRetryLogic(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw);
void SwingAndShoot(ros::Publisher &pub, double swing_speed, double swing_angle, int swing_times);
void Turn_safe_1(ros::Publisher &pub, double angular_z, double distance);

// ---------- 里程计反馈：用于卡死检测 ----------
double g_x   = 0.0;
double g_y   = 0.0;
double g_yaw = 0.0;

double normAngle(double a)            // 把角度规整到 (-PI, PI]
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    g_x = msg->pose.pose.position.x;
    g_y = msg->pose.pose.position.y;
    const geometry_msgs::Quaternion &q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    g_yaw = atan2(siny_cosp, cosy_cosp);
}

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

// ---------------------------------------------------------------------------
// 带"卡死自恢复"的导航。
// 导航途中如果小车原地不动/缓慢磨蹭（净位移、净转角都很小）超过 STALL_TIMEOUT 秒，
// 就取消导航 -> 原地旋转一圈顶开箱子 -> 重新发目标；超过 MAX_RECOVER 次仍失败则放弃该点。
// ---------------------------------------------------------------------------
bool Move2goal(MoveBaseClient &ac, ros::Publisher &pub, double x, double y, double yaw)
{
    // ===== 可调参数 =====
    const double STALL_TIMEOUT = 3.0;          // 卡住多久(秒)触发脱困
    const double STALL_DIST    = 0.05;         // 3秒内净位移<5cm 视为卡住
    const double STALL_YAW     = 0.17;         // 3秒内净转角<10°(0.17rad) 视为卡住
    const double NEAR_GOAL     = 0.15;         // 距目标<15cm时不判卡死(防止在终点误打转)
    const double SPIN_SPEED    = 0.8;          // 脱困原地旋转角速度 rad/s（想更快脱困可调大，如2.0）
    const double SPIN_ANGLE    = 2.0 * M_PI;   // 脱困旋转角度(360°)，想省时间改 M_PI(180°)
    const int    MAX_RECOVER   = 3;            // 最多脱困次数，超过则放弃该点
    // =====================

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);

    int recover_count = 0;

    while (ros::ok() && recover_count <= MAX_RECOVER)
    {
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
        ROS_INFO("MoveBase Send Goal to (%.3f, %.3f, %.3f) [recover=%d]", x, y, yaw, recover_count);

        // ---- 轮询等待 + 卡死检测（不再用阻塞的 waitForResult）----
        ros::Rate rate(20);
        ros::Time last_progress = ros::Time::now();
        double last_x = g_x, last_y = g_y, last_yaw = g_yaw;
        bool stalled = false;

        while (ros::ok())
        {
            if (ac.getState().isDone()) break;      // 导航已结束(成功/失败/取消)

            // 和"上次有进展"的位置比，判断是否还在有效移动
            double dx = g_x - last_x, dy = g_y - last_y;
            double dist = sqrt(dx * dx + dy * dy);
            double dyaw = fabs(normAngle(g_yaw - last_yaw));
            if (dist > STALL_DIST || dyaw > STALL_YAW)
            {
                last_progress = ros::Time::now();   // 有进展：刷新基准
                last_x = g_x; last_y = g_y; last_yaw = g_yaw;
            }

            double dist_to_goal = sqrt((g_x - x) * (g_x - x) + (g_y - y) * (g_y - y));
            if (dist_to_goal >= NEAR_GOAL &&
                (ros::Time::now() - last_progress).toSec() > STALL_TIMEOUT)
            {
                stalled = true;
                ROS_WARN("STALL detected (no progress > %.1fs), spinning to escape...", STALL_TIMEOUT);
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }

        if (!stalled)
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Reached (%.3f, %.3f)!", x, y);
                return true;
            }
            // 失败但不是卡死：后退一点再重发目标
            ROS_WARN("Navigation aborted, back up and retry");
            ac.cancelAllGoals();
            Move_safe(pub, -0.1, 0.0, 10);          // 后退约 0.1m
            recover_count++;
            continue;
        }

        // ===== 卡死脱困：取消导航，原地旋转顶开箱子，然后重新导航 =====
        ac.cancelAllGoals();
        ros::Duration(0.1).sleep();
        int spin_counts = (int)round((SPIN_ANGLE / SPIN_SPEED) * 10.0);  // Turn_safe_1 是 10Hz 计数
        Turn_safe_1(pub, SPIN_SPEED, spin_counts);
        recover_count++;
        ROS_INFO("Escape spin done, re-navigating (attempt %d)...", recover_count);
    }

    ROS_ERROR("Give up reaching (%.3f, %.3f) after %d recoveries", x, y, MAX_RECOVER);
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shoot_robot_base");
    ros::NodeHandle nh;

    geometry_msgs::Twist vel_msg;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);   // 卡死检测用
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

    // =======================================================================
    // 规则：每个点导航成功(Move2goal返回true)才打靶；
    //       卡住脱困超过 MAX_RECOVER 次仍失败(返回false)就跳过该点，直接去下一个点。
    //       "中转"动作按它所属点的位姿校准，所以和该点放在同一个 if 里。
    // =======================================================================

    // First target point
    if (Move2goal(ac, pub, 0.795, -0.80, -1.068))
    {
        SwingAndShoot(pub, 0.15, 25.0, 1);
        Turn_safe_1(pub, 1.5708, 13);
    }
    else { ROS_WARN("Point 1 unreachable, skip shooting, go next."); }

    // Second target point
    if (Move2goal(ac, pub, 0.9838, 1.487, 0.657))
    {
        SwingAndShoot(pub, 0.15, 25.0, 1);
        Turn_safe_1(pub, 1.5708, 13);
    }
    else { ROS_WARN("Point 2 unreachable, skip shooting, go next."); }

    // Third target point
    if (Move2goal(ac, pub, 0.09, 1.50, 1.975))
    {
        SwingAndShoot(pub, 0.15, 25.0, 1);
        Turn_safe_1(pub, 1.5708, 13);
    }
    else { ROS_WARN("Point 3 unreachable, skip shooting, go next."); }

    // Fourth target point
    if (Move2goal(ac, pub, 0.044, 0.855, -2.056))
    {
        // Turn_safe_1(pub, 0.4998, 10);       // 左转 28.64°，车头对准第四个点
        //Move_safe(pub, -0.2439, 0.0, 25);    // 前进 0.6098 m（60.98 cm）
        // Turn_safe_1(pub, -0.4671, 6);       // 右转 16.06°，转到要求的朝向 -1.976 rad
        SwingAndShoot(pub, 0.10, 20.0, 1);

        Move_safe(pub, -0.20, 0.0, 15);
        Turn_safe_1(pub, 0.5236, 26);
        Move_safe(pub, 0.25, 0.0, 20);
        Turn_safe_1(pub, -0.5236, 15);
    }
    else { ROS_WARN("Point 4 unreachable, skip shooting, go next."); }

    // Fifth target point
    if (Move2goal(ac, pub, 1.600, -0.8704, -2.5077))
    {
        SwingAndShoot(pub, 0.15, 25.0, 1);
        Turn_safe_1(pub, 1.5708, 13);
    }
    else { ROS_WARN("Point 5 unreachable, skip shooting, go next."); }

    // Sixth target point
    if (Move2goal(ac, pub, 2.386, -0.9004, -1.167))
    {
        SwingAndShoot(pub, 0.15, 25.0, 1);
        Turn_safe_1(pub, 1.5708, 13);
    }
    else { ROS_WARN("Point 6 unreachable, skip shooting, go next."); }

    // Seventh target point
    if (Move2goal(ac, pub, 2.509, -0.304, 1.041))
    {
        SwingAndShoot(pub, 0.10, 20.0, 1);

        Move_safe(pub, -0.20, 0.0, 15);
        Turn_safe_1(pub, 0.5236, 19);
        Move_safe(pub, 0.25, 0.0, 25);
        Turn_safe_1(pub, -0.5236, 8);
        //Move_safe(pub, 0.25, 0.0, 20);
    }
    else { ROS_WARN("Point 7 unreachable, skip shooting, go next."); }

    // Eighth target point
    if (Move2goal(ac, pub, 1.70, 1.48, 2.199))
    {
        SwingAndShoot(pub, 0.15, 25.0, 1);
        Turn_safe_1(pub, -0.5236, 25);
    }
    else { ROS_WARN("Point 8 unreachable, skip shooting, go next."); }

    // Ninth target point
    if (Move2goal(ac, pub, 2.460, 1.310, 0.843))
    {
        SwingAndShoot(pub, 0.15, 20.0, 9);
    }
    else { ROS_WARN("Point 9 unreachable, skip shooting, go next."); }

    // 【修改】完成所有动作，返回起始点后，关闭激光
    ros::service::waitForService("/close");
    shoot_close_client.call(empty_srv);
    ROS_INFO("Returned to start. Laser OFF.");
    return 0;
}
