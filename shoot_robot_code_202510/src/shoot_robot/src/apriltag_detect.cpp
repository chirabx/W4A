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

    // 控制参数（比赛实测版：Kp=3 + 死区防蛇形）
    double Kp = 3.0;                        // 比例系数（实测3最佳，过大会蛇形抖动）
    const double target_x_tolerance = 0.01; // x容差 1cm
    const double z_target_distance = 0.114; // z目标距离 11.4cm
    const double target_z_tolerance = 0.02; // z容差 2cm
    double deadband_x = 0.005;              // x转向死区 5mm：|x|<5mm 不转向
    double deadband_z = 0.005;              // z前后死区 5mm：|z_err|<5mm 不前进/后退

    // 状态机：SEARCHING=搜索tag  ADJUSTING=已初射，微调中
    enum State { SEARCHING, ADJUSTING };
    State state_ = SEARCHING;

    bool should_exit_ = false;
    bool is_backing_up_ = false;
    ros::Time backup_start_time_;
    const double backup_duration_ = 2.0;    // 搜索后退超时

    ros::Time adjust_start_time_;           // 微调开始时刻
    const double adjust_timeout_ = 3.0;     // 微调超时2秒
    bool precise_shot_done_ = false;        // 精射是否已完成（防重复）

    std_srvs::Empty empty_srv;
    int tag_id;

public:
    AprilTagController() : private_nh_("~")
    {
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");

        private_nh_.getParam("tag", tag_id);
        // Kp/死区可经 launch 覆盖：_kp:=3 _deadband_x:=0.005 _deadband_z:=0.005
        private_nh_.param("kp", Kp, 3.0);
        private_nh_.param("deadband_x", deadband_x, 0.005);
        private_nh_.param("deadband_z", deadband_z, 0.005);
        ROS_INFO("The value of tag is %d. Kp=%.2f deadband_x=%.3f deadband_z=%.3f",
                 tag_id, Kp, deadband_x, deadband_z);
    }

    void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
    {
        geometry_msgs::Twist cmd_vel;
        bool target_found = false;

        for (const auto &detection : msg->detections)
        {
            if (detection.id[0] == tag_id)
            {
                double current_x = detection.pose.pose.pose.position.x;
                double current_z = detection.pose.pose.pose.position.z;
                target_found = true;

                // ============================================================
                // 状态1：首次检测到tag → 直接射击，进入微调
                // ============================================================
                if (state_ == SEARCHING)
                {
                    ROS_INFO("Tag %d detected! Immediate shoot! x=%.4f z=%.4f", tag_id, current_x, current_z);
                    shoot_client.call(empty_srv);           // 立刻射一发
                    state_ = ADJUSTING;
                    adjust_start_time_ = ros::Time::now();
                    precise_shot_done_ = false;
                    ROS_INFO("Entering adjustment (2s timeout)...");
                }

                // ============================================================
                // 状态2：微调中 → 同时调x和z（并行，不串行）
                // ============================================================
                if (state_ == ADJUSTING)
                {
                    bool x_ok = fabs(current_x) < target_x_tolerance;
                    bool z_ok = fabs(current_z - z_target_distance) < target_z_tolerance;

                    // 并行调x（转向）+ z（前后），互不阻塞（不再串行）
                    if (!x_ok)
                    {
                        if (fabs(current_x) < deadband_x)
                        {
                            cmd_vel.angular.z = 0;   // 死区内：不转向，防蛇形
                        }
                        else
                        {
                            // 近场用小增益防超调，远场用Kp
                            double gain = (fabs(current_x) < 0.03) ? 2.0 : Kp;
                            cmd_vel.angular.z = gain * (-current_x);
                        }
                    }
                    if (!z_ok)
                    {
                        if (fabs(current_z - z_target_distance) < deadband_z)
                        {
                            cmd_vel.linear.x = 0;    // 死区内：不前后，防蛇形
                        }
                        else
                        {
                            cmd_vel.linear.x = Kp * 0.3 * (current_z - z_target_distance);
                        }
                    }

                    // x和z都对准了 → 精确射击（只射一次，防重复）
                    if (x_ok && z_ok && !precise_shot_done_)
                    {
                        ROS_INFO("Aligned! Precise shot! x=%.4f z=%.4f", current_x, current_z);
                        shoot_client.call(empty_srv);
                        precise_shot_done_ = true;
                        doPostShootAction();
                        return;
                    }

                    // 2秒超时 → 不管对没对准，执行射后动作
                    ros::Duration elapsed = ros::Time::now() - adjust_start_time_;
                    if (elapsed.toSec() >= adjust_timeout_)
                    {
                        ROS_INFO("Timeout %.1fs! x=%.4f z=%.4f. Post-shoot anyway.",
                                 elapsed.toSec(), current_x, current_z);
                        doPostShootAction();
                        return;
                    }

                    ROS_INFO("Adjusting %.1fs | x=%.4f(%s) z=%.4f(%s)",
                             elapsed.toSec(),
                             current_x, x_ok ? "OK" : "ADJ",
                             current_z, z_ok ? "OK" : "ADJ");
                }
                break;
            }
        }

        // ================================================================
        // 未找到tag
        // ================================================================
        if (!target_found)
        {
            shoot_client.call(empty_srv);
            if (state_ == ADJUSTING)
            {
                // 微调中丢失tag → 检查超时
                ros::Duration elapsed = ros::Time::now() - adjust_start_time_;
                if (elapsed.toSec() >= adjust_timeout_)
                {
                    ROS_INFO("Tag lost + timeout. Post-shoot.");
                    doPostShootAction();
                    return;
                }
                ROS_INFO("Tag lost during adjust, waiting %.1fs...", elapsed.toSec());
                // 不发速度，原地等tag重新出现
            }
            else // SEARCHING
            {
                if (!is_backing_up_)
                {
                    is_backing_up_ = true;
                    backup_start_time_ = ros::Time::now();
                    ROS_INFO("Tag not found, backing up...");
                }

                ros::Duration backup_elapsed = ros::Time::now() - backup_start_time_;
                if (backup_elapsed.toSec() >= backup_duration_)
                {
                    ROS_INFO("Backup timeout, giving up this point.");
                    executeNextTask();
                    ros::param::set("/apriltag_exit_status", "unnormal_exit");
                    return;
                }

                cmd_vel.linear.x = -0.05;
                cmd_vel.angular.z = 0;
            }
        }
        else
        {
            if (is_backing_up_)
            {
                is_backing_up_ = false;
                ROS_INFO("Tag found, stop backup.");
            }
        }

        cmd_vel_pub_.publish(cmd_vel);
    }

    // ========================================================
    // 射后动作：右转→左转→后退→退出（保持原逻辑）
    // ========================================================
    void doPostShootAction()
    {
        geometry_msgs::Twist cmd_vel;
        ros::Rate loop_rate(10);

        // 停稳
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmd_vel_pub_.publish(cmd_vel);

        // 向右 0.5s
        cmd_vel.angular.z = -0.1;
        int count = 0;
        while (ros::ok() && count < 2)
        {
            cmd_vel_pub_.publish(cmd_vel);
            loop_rate.sleep();
            count++;
        }

        // 向左 1.0s
        cmd_vel.angular.z = 0.1;
        count = 0;
        while (ros::ok() && count < 3)
        {
            cmd_vel_pub_.publish(cmd_vel);
            loop_rate.sleep();
            count++;
        }

        // 后退 1.0s
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = -0.07;
        count = 0;
        while (ros::ok() && count < 10)
        {
            cmd_vel_pub_.publish(cmd_vel);
            loop_rate.sleep();
            count++;
        }

        // 停车
        cmd_vel.linear.x = 0.0;
        cmd_vel_pub_.publish(cmd_vel);

        should_exit_ = true;
        ros::param::set("/apriltag_exit_status", "normal_exit");
        ros::shutdown();
    }

    void executeNextTask()
    {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0;
        stop_cmd.angular.z = 0;
        cmd_vel_pub_.publish(stop_cmd);

        should_exit_ = true;
        ros::shutdown();
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