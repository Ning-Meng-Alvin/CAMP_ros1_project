#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <string>
#include <algorithm>

static inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

class SpacemouseToServo {
public:
  SpacemouseToServo(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // Frames & topics
    pnh_.param<std::string>("base_frame",  base_frame_,  "panda_link0");
    pnh_.param<std::string>("input_topic", input_topic_, "/spacenav/twist");
    pnh_.param<std::string>("output_topic",output_topic_,"/velocity_cmd"); // feed VelocityControl → Servo

    // Deadzone (device units), small to remove micro jitters. Set 0.0 to disable.
    pnh_.param<double>("deadzone_lin", dz_lin_, 0.0);
    pnh_.param<double>("deadzone_ang", dz_ang_, 0.0);

    // Clip to [-1,1] for unitless inputs (recommended for Servo unitless mode)
    pnh_.param<bool>("clip_to_unit", clip_to_unit_, true);

    // Optional watchdog: if no input for T seconds, publish a single zero Twist
    pnh_.param<bool>("enable_watchdog", enable_watchdog_, false);
    pnh_.param<double>("watchdog_timeout_s", watchdog_timeout_s_, 0.3);

    sub_ = nh_.subscribe(input_topic_, 100, &SpacemouseToServo::onRaw, this);
    pub_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_, 100);

    if (enable_watchdog_) {
      watchdog_timer_ = nh_.createTimer(ros::Duration(0.05), &SpacemouseToServo::onWatchdog, this);
    }

    last_msg_time_ = ros::Time::now();
    ROS_INFO_STREAM("spacemouse_to_servo (no-scale/no-filter) started. in=" << input_topic_
                    << ", out=" << output_topic_ << ", base=" << base_frame_);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  ros::Timer      watchdog_timer_;

  std::string base_frame_, input_topic_, output_topic_;
  double dz_lin_{0.0}, dz_ang_{0.0};
  bool clip_to_unit_{true};
  bool enable_watchdog_{false};
  double watchdog_timeout_s_{0.3};
  ros::Time last_msg_time_;

  static inline double deadzone(double x, double dz) {
    return (std::fabs(x) <= dz) ? 0.0 : x;
  }

  void publishStamped(double lx, double ly, double lz, double ax, double ay, double az) {
    if (clip_to_unit_) {
      lx = clamp(lx, -1.0, 1.0);
      ly = clamp(ly, -1.0, 1.0);
      lz = clamp(lz, -1.0, 1.0);
      ax = clamp(ax, -1.0, 1.0);
      ay = clamp(ay, -1.0, 1.0);
      az = clamp(az, -1.0, 1.0);
    }
    geometry_msgs::TwistStamped o;
    o.header.stamp = ros::Time::now();
    o.header.frame_id = base_frame_;
    o.twist.linear.x  = lx;
    o.twist.linear.y  = ly;
    o.twist.linear.z  = lz;
    o.twist.angular.x = ax;
    o.twist.angular.y = ay;
    o.twist.angular.z = az;
    pub_.publish(o);
  }

  void onRaw(const geometry_msgs::Twist::ConstPtr& m) {
    // Minimal processing: deadzone only, then pass-through (no scaling, no filtering)
    const double lx = deadzone(m->linear.x,  dz_lin_);
    const double ly = deadzone(m->linear.y,  dz_lin_);
    const double lz = deadzone(m->linear.z,  dz_lin_);
    const double ax = deadzone(m->angular.x, dz_ang_);
    const double ay = deadzone(m->angular.y, dz_ang_);
    const double az = deadzone(m->angular.z, dz_ang_);

    publishStamped(lx, ly, lz, ax, ay, az);
    last_msg_time_ = ros::Time::now();
  }

  void onWatchdog(const ros::TimerEvent&) {
    if (!enable_watchdog_) return;
    const ros::Time now = ros::Time::now();
    if ((now - last_msg_time_).toSec() > watchdog_timeout_s_) {
      // Publish a single zero command to halt
      publishStamped(0,0,0, 0,0,0);
      last_msg_time_ = now; // avoid spamming
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "spacemouse_to_servo_node");
  ros::NodeHandle nh, pnh("~");
  SpacemouseToServo node(nh, pnh);
  ros::spin();
  return 0;
}
