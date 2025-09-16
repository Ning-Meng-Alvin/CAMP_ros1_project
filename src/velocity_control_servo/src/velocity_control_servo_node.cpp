// velocity_control_servo_node.cpp  
// (unitless-friendly + optional clip + watchdog + low-latency)

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <algorithm>
#include <cmath>
#include <string>

struct Twist6 { double vx{0}, vy{0}, vz{0}, wx{0}, wy{0}, wz{0}; };
static inline double clamp(double x, double lo, double hi){ return std::max(lo, std::min(x, hi)); }

class VelocityControlServoNode {
public:
  VelocityControlServoNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // Topics & frames
    pnh_.param<std::string>("base_frame",   base_frame_,   "panda_link0");
    pnh_.param<std::string>("input_topic",  input_topic_,  "/velocity_cmd");
    pnh_.param<std::string>("output_topic", output_topic_, "/servo_server/delta_twist_cmds");

    // Filters and publishing rate
    pnh_.param<bool>("enable_filter", enable_filter_, false);
    pnh_.param<double>("cutoff_hz",   cutoff_hz_,   20.0);
    pnh_.param<double>("pub_rate_hz", pub_rate_hz_, 400.0);   // 100–400 Hz

    // Physical limits & slew rate
    pnh_.param<double>("max_linear",        max_linear_,        1.0); // m/s  
    pnh_.param<double>("max_angular",       max_angular_,       1.0); // rad/s 
    pnh_.param<double>("max_linear_slew",   max_linear_slew_,   8.0); // m/s^2 
    pnh_.param<double>("max_angular_slew",  max_angular_slew_, 20.0); // rad/s^2 

    // Watchdog
    pnh_.param<double>("timeout_s",        timeout_s_,        0.15);
    pnh_.param<bool>("zero_on_timeout",    zero_on_timeout_,  true);

    // Optional clipping [-1,1]
    pnh_.param<bool>("clip_to_unit",       clip_to_unit_,     true);  
    pnh_.param<bool>("immediate_publish",  immediate_pub_,    true); // publish immediately on new msg

    sub_   = nh_.subscribe(input_topic_, 100, &VelocityControlServoNode::onIn, this);
    pub_   = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_, 50);
    timer_ = nh_.createTimer(ros::Duration(1.0 / pub_rate_hz_), &VelocityControlServoNode::onTimer, this);

    last_pub_time_ = ros::Time::now();
    last_msg_time_ = ros::Time(0);
    have_msg_ = false;

    ROS_INFO_STREAM("velocity_control_servo_node started. base=" << base_frame_
                    << ", in=" << input_topic_ << ", out=" << output_topic_
                    << ", rate=" << pub_rate_hz_ << "Hz");
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  ros::Timer      timer_;

  std::string base_frame_, input_topic_, output_topic_;
  bool   enable_filter_{false};
  double cutoff_hz_{20.0}, pub_rate_hz_{400.0};
  double max_linear_{0.25}, max_angular_{1.0};
  double max_linear_slew_{4.0}, max_angular_slew_{10.0};
  double timeout_s_{0.4};
  bool   zero_on_timeout_{true};
  bool   clip_to_unit_{true};
  bool   immediate_pub_{false};

  Twist6 target_{}, filtered_{}, last_pub_{};
  ros::Time last_msg_time_, last_pub_time_;
  bool have_msg_{false};

  void onIn(const geometry_msgs::TwistStamped::ConstPtr& msg){
    // Optionally clip to [-1,1], then apply physical (or unitless) limits
    auto cap_lin = [&](double v){ if (clip_to_unit_) v = clamp(v, -1.0, 1.0); return clamp(v, -max_linear_,  max_linear_); };
    auto cap_ang = [&](double v){ if (clip_to_unit_) v = clamp(v, -1.0, 1.0); return clamp(v, -max_angular_, max_angular_); };

    target_.vx = cap_lin(msg->twist.linear.x);
    target_.vy = cap_lin(msg->twist.linear.y);
    target_.vz = cap_lin(msg->twist.linear.z);
    target_.wx = cap_ang(msg->twist.angular.x);
    target_.wy = cap_ang(msg->twist.angular.y);
    target_.wz = cap_ang(msg->twist.angular.z);

    if (!have_msg_) {
      filtered_ = target_;
      last_pub_ = target_;
      have_msg_ = true;
    }
    last_msg_time_ = ros::Time::now();

    if (immediate_pub_) publishNow(last_msg_time_, target_); // low latency: publish one frame immediately
  }

  void lowPass(Twist6& y, const Twist6& u, double dt){
    if (!enable_filter_) { y = u; return; }
    if (cutoff_hz_ <= 1e-6) { y = u; return; }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz_);
    const double a = dt / (dt + tau);
    y.vx += a * (u.vx - y.vx);
    y.vy += a * (u.vy - y.vy);
    y.vz += a * (u.vz - y.vz);
    y.wx += a * (u.wx - y.wx);
    y.wy += a * (u.wy - y.wy);
    y.wz += a * (u.wz - y.wz);
  }

  void slewLimit(Twist6& y, const Twist6& y_prev, double dt){
    const double lv = max_linear_slew_  * dt;
    const double la = max_angular_slew_ * dt;
    y.vx = clamp(y.vx, y_prev.vx - lv, y_prev.vx + lv);
    y.vy = clamp(y.vy, y_prev.vy - lv, y_prev.vy + lv);
    y.vz = clamp(y.vz, y_prev.vz - lv, y_prev.vz + lv);
    y.wx = clamp(y.wx, y_prev.wx - la, y_prev.wx + la);
    y.wy = clamp(y.wy, y_prev.wy - la, y_prev.wy + la);
    y.wz = clamp(y.wz, y_prev.wz - la, y_prev.wz + la);
  }

  void publishNow(const ros::Time& t, const Twist6& y){
    geometry_msgs::TwistStamped out;
    out.header.stamp = t;
    out.header.frame_id = base_frame_;
    out.twist.linear.x  = y.vx; out.twist.linear.y  = y.vy; out.twist.linear.z  = y.vz;
    out.twist.angular.x = y.wx; out.twist.angular.y = y.wy; out.twist.angular.z = y.wz;
    pub_.publish(out);
  }

  void onTimer(const ros::TimerEvent&){
    if (!have_msg_) return;

    const ros::Time now = ros::Time::now();
    const double dt = (now - last_pub_time_).toSec();
    if (dt <= 0.0) return;

    Twist6 tgt = target_;
    if (zero_on_timeout_ && (now - last_msg_time_).toSec() > timeout_s_) {
      tgt = Twist6{}; // smoothly decay to zero
    }

    Twist6 y = filtered_;
    lowPass(y, tgt, dt);
    slewLimit(y, last_pub_, dt);

    publishNow(now, y);

    last_pub_time_ = now;
    last_pub_ = y;
    filtered_ = y;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "velocity_control_servo_node");
  ros::NodeHandle nh, pnh("~");
  VelocityControlServoNode node(nh, pnh);
  ros::spin();
  return 0;
}
