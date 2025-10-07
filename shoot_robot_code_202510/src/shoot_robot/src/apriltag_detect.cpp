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
    const double Kp = 8;                    // 比例系数
    const double target_x_tolerance = 0.01; // X轴位置容忍误差

    const double z_target_distance = 0.114;
    const double target_z_tolerance = 0.02;

    bool should_exit_ = false;
    bool is_backing_up_ = false;
    ros::Time backup_start_time_;
    const double backup_duration_ = 5.0; // 后退持续时间（秒）

    std_srvs::Empty empty_srv;

    // 在参数中加载要射击的tag目标
    int tag_id;

public:
    AprilTagController() : private_nh_("~")
    {
        // 初始化订阅者和发布者
        tag_sub_ = nh_.subscribe("tag_detections", 1, &AprilTagController::tagCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        shoot_client = nh_.serviceClient<std_srvs::Empty>("/shoot");

        private_nh_.getParam("tag", tag_id);
        ROS_INFO("The value of tag is %d.", tag_id);
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
                ROS_INFO("The current x position is %f", current_x);
                double current_z = detection.pose.pose.pose.position.z;
                ROS_INFO("The current z position is %f", current_z);

                if ((fabs(current_x) < target_x_tolerance) && (fabs(current_z - z_target_distance) < target_z_tolerance))
                {
                    shoot_client.call(empty_srv);
                    cmd_vel.angular.z = -0.1;
                    cmd_vel.linear.x = 0;
                    int count = 0;
                    ros::Rate loop_rate(10);
                    while (ros::ok() && count < 20)
                    {
                        ROS_INFO("shoot");
                        cmd_vel_pub_.publish(cmd_vel);
                        ros::spinOnce();
                        loop_rate.sleep();
                        count++;
                    }

                    should_exit_ = true;
                    ros::shutdown(); // 终止ROS通信
                    return;          // 直接退出回调函数
                }

                else if (fabs(current_x) > target_x_tolerance)
                {
                    cmd_vel.angular.z = Kp * (-current_x);
                }
                else if (fabs(current_z - z_target_distance) > target_z_tolerance)
                {
                    cmd_vel.linear.x = Kp * 0.3 * (current_z - z_target_distance);
                }
                target_found = true;
                break;
            }
        }
        if (!target_found)
        {
            // 如果还没有开始后退，记录开始时间
            if (!is_backing_up_)
            {
                is_backing_up_ = true;
                backup_start_time_ = ros::Time::now();
                ROS_INFO("Starting backup, target tag not detected");
            }

            // 检查是否已经后退足够时间
            ros::Duration backup_elapsed = ros::Time::now() - backup_start_time_;
            if (backup_elapsed.toSec() >= backup_duration_)
            {
                ROS_INFO("Backup time reached, executing next task");
                executeNextTask();
                return;
            }

            // 继续后退
            ROS_INFO("Backing up... %.1f seconds elapsed", backup_elapsed.toSec());
            cmd_vel.linear.x = -0.1;
            cmd_vel.angular.z = 0;
        }
        else
        {
            // 如果检测到目标，重置后退状态
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
        ROS_INFO("Executing next task...");

        // 停止机器人运动
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0;
        stop_cmd.angular.z = 0;
        cmd_vel_pub_.publish(stop_cmd);

        // 这里可以添加执行下个任务的具体逻辑
        // 例如：发布任务完成消息、调用其他服务、或退出程序等

        // 示例：退出程序
        should_exit_ = true;
        ros::shutdown();

        ROS_INFO("Next task execution completed");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_controller");
    AprilTagController controller;
    // 非阻塞式循环
    ros::Rate loop_rate(10); // 控制循环频率（10Hz）
    while (ros::ok())
    {
        ros::spinOnce(); // 处理回调队列
        loop_rate.sleep();
    }
    // 退出前的清理工作（可选）
    ROS_INFO("Node shutdown gracefully");
    return 0;
}
