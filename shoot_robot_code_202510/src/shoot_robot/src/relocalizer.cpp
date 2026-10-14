#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <algorithm>

class Relocalizer
{
public:
  Relocalizer() : nh_(), pnh_("~"), tf_(ros::Duration(10.0)), relocalizing_(false)
  {
    // ---------- 参数 ----------
    pnh_.param<std::string>("map_frame",  map_frame_,  "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("map_topic",  map_topic_,  "/map");
    pnh_.param<std::string>("imu_topic",  imu_topic_,  "/imu/data");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<std::string>("cmd_topic",  cmd_topic_,  "/cmd_vel");

    pnh_.param<double>("match_bad",     match_bad_,     0.30);  // 低于此分判定偏移
    pnh_.param<double>("match_good",    match_good_,    0.55);  // 高于此分认为重定位成功
    pnh_.param<int>   ("bad_count",     bad_trigger_,   8);     // 连续多少次低分才触发
    pnh_.param<double>("search_radius", search_radius_, 0.80);  // 本地搜索半径(m)
    pnh_.param<double>("search_yaw",    search_yaw_,    0.50);  // 本地搜索角度范围(rad)
    pnh_.param<double>("search_step",   search_step_,   0.10);  // 平移步长(m)
    pnh_.param<double>("imu_spike",     imu_spike_thr_, 1.50);  // IMU 冲击阈值(rad/s)
    pnh_.param<double>("cooldown",      cooldown_,      8.0);   // 重定位后冷却(s)
    pnh_.param<double>("still_vel",     still_vel_,     0.08);  // 判静止的速度阈值(m/s)

    // ---------- 通信 ----------
    map_sub_  = nh_.subscribe(map_topic_,  1,  &Relocalizer::mapCb,  this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1,  &Relocalizer::scanCb, this);
    imu_sub_  = nh_.subscribe(imu_topic_,  50, &Relocalizer::imuCb,  this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &Relocalizer::odomCb, this);

    initpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    cmd_pub_      = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
    status_pub_   = nh_.advertise<std_msgs::String>("/reloc/status", 1);

    clear_srv_  = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    global_srv_ = nh_.serviceClient<std_srvs::Empty>("/global_localization");
    trigger_srv_ = pnh_.advertiseService("relocalize", &Relocalizer::triggerCb, this);
    check_srv_   = nh_.advertiseService("/reloc/check_fix", &Relocalizer::checkFixCb, this);

    monitor_ = nh_.createTimer(ros::Duration(0.2), &Relocalizer::monitorCb, this);

    bad_count_ = 0;
    last_score_ = -1.0;
    last_trigger_time_ = ros::Time(0);
    last_imu_spike_time_ = ros::Time(0);
    odom_vel_ = 0.0;

    ROS_WARN("[RELOC] ready: bad<%.2f good>%.2f radius=%.2fm yaw=%.2frad",
             match_bad_, match_good_, search_radius_, search_yaw_);
  }

  // ---------- 回调 ----------
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr &m) { map_ = *m; }
  void scanCb(const sensor_msgs::LaserScan::ConstPtr &s) { scan_ = *s; }

  void odomCb(const nav_msgs::Odometry::ConstPtr &o)
  {
    odom_vel_ = std::abs(o->twist.twist.linear.x) + std::abs(o->twist.twist.linear.y);
  }

  void imuCb(const sensor_msgs::Imu::ConstPtr &msg)
  {
    double wz = msg->angular_velocity.z;
    if (std::abs(wz) > imu_spike_thr_)
    {
      ros::Time now = ros::Time::now();
      // 0.5s 内的连续抖动只算一次撞击,避免抖一下刷一堆日志
      if (last_imu_spike_time_.is_zero() ||
          (now - last_imu_spike_time_).toSec() > 0.5)
      {
        last_imu_spike_time_ = now;
        ROS_WARN("[DETECT] IMU impact: |wz|=%.3f rad/s", wz);
      }
    }
  }

  bool triggerCb(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    ROS_WARN("[RELOC] manual trigger");
    doRelocalize();
    return true;
  }

  // ===== 主动查询:撞了就重定位,没撞就什么都不做 =====
  // 主程序在 Move_safe 之后调用这个 service,然后读 /reloc/offset 参数
  bool checkFixCb(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    relocalizing_ = true;
    stopRobot();

    double x, y, yaw;
    if (!getCurrentPose(x, y, yaw))
    {
      ROS_ERROR("[CHECK] no tf pose, abort");
      relocalizing_ = false;
      return false;
    }

    // 判据是"扫描和地图还对不对得上",不是 IMU 撞没撞:
    // 撞到对方车(动态物)但自己没被撞位移 → 分数正常 → 不重定位
    double s; int h, t;
    bool ok = scorePose(x, y, yaw, s, h, t);
    double score = ok ? s : 0.0;

    if (ok && score >= match_good_)
    {
      ROS_INFO("[CHECK] score=%.2f -> no offset, skip relocalize", score);
      ros::param::set("/reloc/offset", false);
      relocalizing_ = false;
      return true;                       // 没撞,什么都不做
    }

    ROS_WARN("[CHECK] score=%.2f -> offset detected, relocalizing!", score);
    ros::param::set("/reloc/offset", true);
    doRelocalize();                      // 内部管理 relocalizing_
    ros::param::set("/reloc/offset", true);
    return true;
  }

  // ---------- 监控主循环 ----------
  void monitorCb(const ros::TimerEvent &)
  {
    if (map_.data.empty() || scan_.ranges.empty()) return;   // 还没拿到地图/扫描
    if (relocalizing_) return;

    // 车在动就先不判定——运动中的扫描是拖影,评分不可信
    if (odom_vel_ > still_vel_) { bad_count_ = 0; return; }

    double x, y, yaw;
    if (!getCurrentPose(x, y, yaw)) return;

    double s; int h, t;
    if (!scorePose(x, y, yaw, s, h, t)) return;
    last_score_ = s;

    if (s < match_bad_) bad_count_++;
    else                bad_count_ = 0;

    // IMU 刚被撞过 → 立刻验证一次,不用等累计次数
    bool imu_recent = !last_imu_spike_time_.is_zero() &&
                      (ros::Time::now() - last_imu_spike_time_).toSec() < 1.5;

    if (bad_count_ >= bad_trigger_)
    {
      ROS_WARN("[DETECT] map offset: score=%.2f (%d/%d) bad_count=%d",
               s, h, t, bad_count_);
      doRelocalize();
    }
    else if (imu_recent && s < match_bad_)
    {
      ROS_WARN("[DETECT] impact + low score=%.2f -> relocalize", s);
      doRelocalize();
    }
  }

  // ---------- 核心:扫描-地图匹配评分 ----------
  // 把激光束端点按假设位姿投影到地图系,统计有多少端点落在"墙"上
  bool scorePose(double x, double y, double yaw,
                 double &score, int &hits, int &total)
  {
    const double res = map_.info.resolution;
    const int W = map_.info.width, H = map_.info.height;
    const double ox = map_.info.origin.position.x;
    const double oy = map_.info.origin.position.y;
    const int8_t *d = map_.data.data();
    if (W <= 0 || H <= 0) return false;

    const double cy = cos(yaw), sy = sin(yaw);
    const int    TOL  = 2;                 // ±2 格 = 10cm 容差
    const int    STEP = 3;                 // 每 3 束取一束,降计算量
    const double RMAX = 8.0;

    hits = 0; total = 0;
    for (size_t i = 0; i < scan_.ranges.size(); i += STEP)
    {
      float r = scan_.ranges[i];
      if (!std::isfinite(r) || r < scan_.range_min + 0.05 ||
          r > std::min(RMAX, (double)scan_.range_max)) continue;

      double a  = scan_.angle_min + i * scan_.angle_increment;
      double px = r * cos(a), py = r * sin(a);          // 激光系(≈base 系)
      double mx = x + cy * px - sy * py;                // 地图系
      double my = y + sy * px + cy * py;

      int ci = (int)((mx - ox) / res);
      int cj = (int)((my - oy) / res);
      if (ci < 0 || cj < 0 || ci >= W || cj >= H) continue;  // 地图外,不计

      total++;
      bool found = false;
      for (int dj = -TOL; dj <= TOL && !found; dj++)
        for (int di = -TOL; di <= TOL && !found; di++)
        {
          int ii = ci + di, jj = cj + dj;
          if (ii < 0 || jj < 0 || ii >= W || jj >= H) continue;
          if (d[jj * W + ii] >= 50) found = true;       // 占用格
        }
      if (found) hits++;
    }
    if (total < 10) return false;                       // 有效束太少,不可信
    score = (double)hits / total;
    return true;
  }

  // ---------- 重定位主流程 ----------
  void doRelocalize()
  {
    relocalizing_ = true;
    last_trigger_time_ = ros::Time::now();
    publishStatus("relocalizing");
    ROS_WARN("[RELOC] ===== start =====");

    // 1) 先停车,否则后面所有评分都是拖影
    stopRobot();

    // 2) 本地网格搜索
    double bx = 0, by = 0, byaw = 0, bs = -1;
    bool ok = localSearch(bx, by, byaw, bs);
    if (ok)
      ROS_WARN("[RELOC] local best: score=%.2f  (%.2f, %.2f, %.1f deg)",
               bs, bx, by, byaw * 180.0 / M_PI);

    if (ok && bs >= match_good_)
    {
      publishInitialPose(bx, by, byaw);
      clearCostmaps();
      ROS_WARN("[RELOC] done (local search)");
      publishStatus("done_local");
    }
    else if (!ok || bs < match_bad_)
    {
      // 本地也找不到好解 —— 但**不调用 global_localization**。
      // 原因:对称场地(四个隔间)下 AMCL 全局重定位会收敛到隔壁隔间,
      // 比不重定位更糟。这里只发布本地次优解 + 原地慢转,让 AMCL 用
      // 多视角激光自己精修,同时靠 motion model 维持大致区域不变。
      ROS_WARN("[RELOC] local score=%.2f not good, do in-place scan fix", bs);

      if (ok)
      {
        publishInitialPose(bx, by, byaw);   // 至少把位置掰回局部最优
      }

      // 原地慢转 4 秒:AMCL 边转边用激光匹配,粒子在局部重新收敛
      // 注意转的角度要小 —— 转太快会让 AMCL 的 motion model 把
      // 位姿估计甩出去,反而更容易跳区域
      ros::Rate r(10);
      geometry_msgs::Twist vel;
      ros::Time t0 = ros::Time::now();
      double total_yaw = 0.0;
      double last_yaw = 0.0;
      bool got_last = getCurrentPose(bx, by, last_yaw);
      (void)got_last;

      while (ros::ok() && (ros::Time::now() - t0).toSec() < 4.0)
      {
        vel.angular.z = 0.2;      // 慢转:0.2 rad/s × 4s ≈ 46°
        cmd_pub_.publish(vel);
        r.sleep();
        double cx, cy, cyaw;
        if (getCurrentPose(cx, cy, cyaw))
        {
          double d = std::abs(cyaw - last_yaw);
          if (d > M_PI) d = 2.0 * M_PI - d;
          total_yaw += d;
          last_yaw = cyaw;
          if (total_yaw > M_PI) break;   // 转过 180° 就停,给足视角又不越界
        }
      }
      vel.angular.z = 0.0;
      cmd_pub_.publish(vel);
      clearCostmaps();

      // 等 AMCL 收敛后验证
      for (int i = 0; i < 30 && ros::ok(); ++i)
      {
        ros::Duration(0.2).sleep();
        double x, y, yaw, s; int h, t;
        if (getCurrentPose(x, y, yaw) && scorePose(x, y, yaw, s, h, t))
        {
          ROS_WARN("[RELOC] after scan fix: score=%.2f", s);
          if (s >= match_good_) break;
        }
      }
      publishStatus("done_scanfix");
    }

    ros::Duration(1.0).sleep();      // 等 AMCL 收敛
    bad_count_ = 0;
    relocalizing_ = false;
    ROS_WARN("[RELOC] ===== end =====");
  }

private:

  bool getCurrentPose(double &x, double &y, double &yaw)
  {
    try {
      tf::StampedTransform t;
      tf_.lookupTransform(map_frame_, base_frame_, ros::Time(0), t);
      x = t.getOrigin().x(); y = t.getOrigin().y();
      yaw = tf::getYaw(t.getRotation());
      return true;
    } catch (tf::TransformException &ex) {
      return false;
    }
  }

  void stopRobot()
  {
    geometry_msgs::Twist z;
    for (int i = 0; i < 3 && ros::ok(); ++i)
    {
      cmd_pub_.publish(z);
      ros::Duration(0.05).sleep();
    }
  }

  // 在当前位置附近做 (x, y, yaw) 网格搜索,找匹配分最高的位姿
  bool localSearch(double &bx, double &by, double &byaw, double &bs)
  {
    double x0, y0, yaw0;
    if (!getCurrentPose(x0, y0, yaw0))
    {
      ROS_ERROR("[RELOC] no tf pose, cannot search");
      return false;
    }
    ROS_WARN("[RELOC] searching around (%.2f, %.2f, %.1f deg) r=%.2fm",
             x0, y0, yaw0 * 180.0 / M_PI, search_radius_);

    const int NY = 5;                       // yaw 采样点数
    const double yaw_step = 2.0 * search_yaw_ / (NY - 1);

    bs = -1.0;
    for (int k = 0; k < NY && ros::ok(); ++k)
    {
      double dyaw = -search_yaw_ + k * yaw_step;
      for (double dx = -search_radius_; dx <= search_radius_ + 1e-6; dx += search_step_)
        for (double dy = -search_radius_; dy <= search_radius_ + 1e-6; dy += search_step_)
        {
          double s; int h, t;
          if (!scorePose(x0 + dx, y0 + dy, yaw0 + dyaw, s, h, t)) continue;
          if (s > bs) { bs = s; bx = x0 + dx; by = y0 + dy; byaw = yaw0 + dyaw; }
        }
    }
    if (bs < 0.0) return false;

    // ---------- 阶段2:在粗最优附近细搜索 ----------
    // 粗搜索步长 0.1m / 0.25rad,最优解可能落在网格点中间,
    // 定位会差半个步长(5cm / 7°)。细搜索步长缩到 1/4,在粗最优
    // 周围 ±1 个粗步长内精修,把精度提到 2.5cm / 2.9°。
    ROS_WARN("[RELOC] coarse best score=%.2f, refining...", bs);
    const double fine_step = search_step_ * 0.25;   // 0.025 m
    const double fine_yaw  = 0.05;                  // ≈2.9°
    const double fine_r    = search_step_;          // 细搜索半径 = 1 个粗步长

    for (double dyaw = -fine_r; dyaw <= fine_r + 1e-6; dyaw += fine_yaw)
      for (double dx = -fine_r; dx <= fine_r + 1e-6; dx += fine_step)
        for (double dy = -fine_r; dy <= fine_r + 1e-6; dy += fine_step)
        {
          double s; int h, t;
          if (!scorePose(bx + dx, by + dy, byaw + dyaw, s, h, t)) continue;
          if (s > bs) { bs = s; bx += dx; by += dy; byaw += dyaw; }
        }
    ROS_WARN("[RELOC] refined best score=%.2f (%.3f, %.3f, %.2f deg)",
             bs, bx, by, byaw * 180.0 / M_PI);
    return true;
  }

  void publishInitialPose(double x, double y, double yaw)
  {
    geometry_msgs::PoseWithCovarianceStamped p;
    p.header.frame_id = map_frame_;
    p.header.stamp = ros::Time::now();
    p.pose.pose.position.x = x;
    p.pose.pose.position.y = y;
    p.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    // 协方差给小一点,表示"这个估计比较可信,AMCL 在附近精修即可"
    for (int i = 0; i < 36; ++i) p.pose.covariance[i] = 0.0;
    p.pose.covariance[0]  = 0.09;   // x
    p.pose.covariance[7]  = 0.09;   // y
    p.pose.covariance[35] = 0.06;   // yaw
    initpose_pub_.publish(p);
    ROS_WARN("[RELOC] /initialpose published");
  }

  void clearCostmaps()
  {
    std_srvs::Empty e;
    if (clear_srv_.waitForExistence(ros::Duration(1.0)))
    {
      clear_srv_.call(e);
      ROS_WARN("[RELOC] costmaps cleared");
    }
    else
      ROS_ERROR("[RELOC] clear_costmaps service not available");
  }

  // 全局兜底:粒子全图撒开 + 原地慢转让 AMCL 多角度看
  // 返回 true 表示收敛了(匹配分恢复)
  bool globalRelocalize()
  {
    std_srvs::Empty e;
    if (global_srv_.waitForExistence(ros::Duration(1.0)))
    {
      global_srv_.call(e);
      ROS_WARN("[RELOC] /global_localization called");
    }

    // 原地慢转 4 秒,给 AMCL 更多视角
    ros::Rate r(10);
    geometry_msgs::Twist vel;
    ros::Time t0 = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t0).toSec() < 4.0)
    {
      vel.angular.z = 0.3;
      cmd_pub_.publish(vel);
      r.sleep();
    }
    vel.angular.z = 0.0;
    cmd_pub_.publish(vel);

    // 等 AMCL 收敛,最多 6 秒
    for (int i = 0; i < 30 && ros::ok(); ++i)
    {
      ros::Duration(0.2).sleep();
      double x, y, yaw;
      if (!getCurrentPose(x, y, yaw)) continue;
      double s; int h, t;
      if (scorePose(x, y, yaw, s, h, t) && s >= match_good_)
      {
        ROS_WARN("[RELOC] global converged: score=%.2f", s);
        clearCostmaps();
        return true;
      }
    }
    ROS_ERROR("[RELOC] global did NOT converge");
    return false;
  }

  void publishStatus(const std::string &s)
  {
    std_msgs::String m;
    m.data = s;
    status_pub_.publish(m);
  }

  // ---------- 成员 ----------
  ros::NodeHandle nh_, pnh_;
  tf::TransformListener tf_;

  ros::Subscriber map_sub_, scan_sub_, imu_sub_, odom_sub_;
  ros::Publisher  initpose_pub_, cmd_pub_, status_pub_;
  ros::ServiceClient clear_srv_, global_srv_;
  ros::ServiceServer trigger_srv_, check_srv_;
  ros::Timer monitor_;

  nav_msgs::OccupancyGrid map_;
  sensor_msgs::LaserScan  scan_;

  std::string map_frame_, base_frame_, scan_topic_, map_topic_,
              imu_topic_, odom_topic_, cmd_topic_;

  double match_bad_, match_good_, search_radius_, search_yaw_, search_step_,
         imu_spike_thr_, cooldown_, still_vel_;
  int    bad_trigger_;

  int    bad_count_;
  double last_score_, odom_vel_;
  bool   relocalizing_;
  ros::Time last_trigger_time_, last_imu_spike_time_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relocalizer");
  Relocalizer r;
  ros::spin();
  return 0;
}
