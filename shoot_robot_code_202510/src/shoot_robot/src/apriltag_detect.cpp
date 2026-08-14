#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

class AprilTagController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber tag_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceClient shoot_client;

    // PID控制参数
    const double Kp = 5;                    // 比例系数
    const double target_x_tolerance = 0.01; // X轴位置容忍误差

    const double z_target_distance = 0.114;
    const double target_z_tolerance = 0.02;

    bool is_backing_up_ = false;
    bool is_shoot_or_down = false;
    ros::Time backup_start_time_;
    const double backup_duration_ = 2.0; // 初始寻找靶子超时时间（秒）

    std_srvs::Empty empty_srv;

    // 目标 tag ID
    int tag_id;

public:
    AprilTagController() : private_nh_("~")
    {
        // 初始化订阅者和发布者
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");

        private_nh_.getParam("tag", tag_id);
        ROS_INFO("The value of target tag is %d.", tag_id);
    }

    // 安全单步旋转函数
    void Turn_safe_1(ros::Publisher &pub, double angular_z, int distance)
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
        // 停止旋转
        vel_msg.angular.z = 0.0;
        pub.publish(vel_msg);
    }

    // 分步射击函数：偏左向左分步旋转射击，偏右向右分步旋转射击
    void StepShoot(ros::Publisher &pub, bool is_left)
    {
        // ===== 参数可调 =====
        const double turn_speed        = 0.18;    // 旋转角速度 rad/s
        const double step_angle        = 0.0697;  // 每次旋转角度（弧度），约 4°
        const int    left_repeat_times  = 3;       // 偏左时左转循环次数
        const int    right_repeat_times = 3;       // 偏右时右转循环次数
        const double pause_sec         = 0.8;     // 每次停顿（射击）时间（秒）
        // =====================
        const int turn_steps = (int)(step_angle / turn_speed / 0.1); // 10Hz 下每步循环次数

        if (is_left)
        {
            ROS_INFO("Target is LEFT. Rotating LEFT (%d times)...", left_repeat_times);
            for (int i = 0; i < left_repeat_times && ros::ok(); i++)
            {
                Turn_safe_1(pub, turn_speed, turn_steps);
                ros::Duration(pause_sec).sleep();
            }
        }
        else
        {
            ROS_INFO("Target is RIGHT. Rotating RIGHT (%d times)...", right_repeat_times);
            for (int i = 0; i < right_repeat_times && ros::ok(); i++)
            {
                Turn_safe_1(pub, -turn_speed, turn_steps);
                ros::Duration(pause_sec).sleep();
            }
        }
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        if (is_shoot_or_down)
        { 
            return; 
        }

        geometry_msgs::Twist cmd_vel;
        bool target_found = false;

        for (const auto &detection : msg->detections)
        {
            if (detection.id[0] == tag_id)
            {
                double current_x = detection.pose.pose.pose.position.x;
                ROS_INFO("The current x position is %f", current_x);
                double current_z = detection.pose.pose.pose.position.z;
                ROS_INFO("The current z position is %f", current_z);

                // 到达射击位置（X对准且Z距离达标）
                if ((fabs(current_x) < target_x_tolerance) && (fabs(current_z - z_target_distance) < target_z_tolerance))
                {
                    is_shoot_or_down = true;
                    // 1. 停车
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_pub_.publish(cmd_vel);

                    // 2. 打开激光
                    shoot_client.call(empty_srv);

                    ROS_INFO("Reached target position. Firing laser and executing step shoot...");

                    // 3. 执行分步扫射打靶
                    if (current_x < 0.0)
                    {
                        StepShoot(cmd_vel_pub_, false);  // 偏左 -> 向左分步射击
                    }
                    else
                    {
                        StepShoot(cmd_vel_pub_, true); // 偏右 -> 向右分步射击
                    }

                    ROS_INFO(">>> Step shoot finished! Exiting immediately to next target. <<<");

                    // // 4. 快速后退微调（后退0.5秒脱离靶位，防止转弯撞靶；若不需要可直接注释）
                    // cmd_vel.linear.x = -0.07;
                    // cmd_vel.angular.z = 0.0;
                    // ros::Rate loop_rate(10);
                    // int count = 0;
                    // while (ros::ok() && count < 5) // 0.5秒
                    // {
                    //     cmd_vel_pub_.publish(cmd_vel);
                    //     loop_rate.sleep();
                    //     count++;
                    // }

                    // 5. 停止
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_pub_.publish(cmd_vel);

                    // 6. 立即设置正常退出状态并关闭节点
                    ros::param::set("/apriltag_exit_status", "normal_exit");
                    ros::shutdown(); // 立即终止当前节点，外部主控脚本捕获后即刻调度下一个导航点
                    return;
                }
                else if (fabs(current_x) > target_x_tolerance)
                {
                    if (fabs(fabs(current_x) - target_x_tolerance) < 0.0065)
                    {
                        cmd_vel.angular.z = 8 * (-current_x);
                    }
                    else
                    {
                        cmd_vel.angular.z = Kp * (-current_x);
                    }
                }
                else if (fabs(current_z - z_target_distance) > target_z_tolerance)
                {
                    cmd_vel.linear.x = Kp * 0.3 * (current_z - z_target_distance);
                }
                target_found = true;
                break;
            }
        }

        if (!target_found && !is_shoot_or_down)
        {
            if (!is_backing_up_)
            {
                is_backing_up_ = true;
                backup_start_time_ = ros::Time::now();
                ROS_INFO("Starting backup, target tag not detected");
            }

            ros::Duration backup_elapsed = ros::Time::now() - backup_start_time_;
            if (backup_elapsed.toSec() >= backup_duration_)
            {
                ROS_INFO("Backup time reached, executing next task");
                executeNextTask();
                ros::param::set("/apriltag_exit_status", "unnormal_exit");
                return;
            }

            ROS_INFO("Backing up... %.1f seconds elapsed", backup_elapsed.toSec());
            cmd_vel.linear.x = -0.05;
            cmd_vel.angular.z = 0;
        }
        else
        {
            if (is_backing_up_)
            {
                is_backing_up_ = false;
                ROS_INFO("Target detected, stopping backup");
            }
        }
        cmd_vel_pub_.publish(cmd_vel);
    }

    void executeNextTask()
    {
        ROS_INFO("Executing next task (timeout fallback)...");

        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0;
        stop_cmd.angular.z = 0;
        cmd_vel_pub_.publish(stop_cmd);

       
        ros::shutdown();

        ROS_INFO("Next task execution completed");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_controller");
    AprilTagController controller;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node shutdown gracefully");
    return 0;
}