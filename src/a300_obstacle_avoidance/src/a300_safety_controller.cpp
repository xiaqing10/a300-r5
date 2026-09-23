#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class A300SafetyController : public rclcpp::Node {
public:
  A300SafetyController():Node("a300_safety_controller") {
    scan_topic_=declare_parameter("scan_topic",std::string("/scan"));
    desired_topic_=declare_parameter("desired_cmd_topic",std::string("/cmd_vel_desired"));
    safe_topic_=declare_parameter("safe_cmd_topic",std::string("/cmd_vel"));
    odom_topic_=declare_parameter("odom_topic",std::string("/odom"));
    warning_=declare_parameter("warning_distance",1.20);
    slow_=declare_parameter("slow_distance",0.80);
    stop_=declare_parameter("stop_distance",0.35);
    front_half_=declare_parameter("front_half_angle_deg",30.0);
    side_min_=declare_parameter("side_angle_min_deg",30.0);
    side_max_=declare_parameter("side_angle_max_deg",100.0);
    vmax_=declare_parameter("max_linear_speed",0.80);
    wmax_=declare_parameter("max_angular_speed",1.00);
    reverse_=declare_parameter("reverse_allowed",false);
    avoid_gain_=declare_parameter("avoid_gain",0.80);
    avoid_min_speed_=declare_parameter("avoid_min_linear",0.10);
    avoid_default_dir_=declare_parameter("avoid_default_dir",1.0);
    avoid_sym_eps_=declare_parameter("avoid_sym_eps",0.03);
    recover_gain_=declare_parameter("recover_gain",0.6);
    recover_damp_=declare_parameter("recover_damp",0.5);
    recover_eps_deg_=declare_parameter("recover_eps_deg",2.0);
    avoid_dir_=0.0; avoid_phase_=0; yaw_=std::numeric_limits<double>::quiet_NaN();
    prev_yaw_=0.0; last_yaw_time_=rclcpp::Time(0); have_yaw_time_=false;
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_,rclcpp::SensorDataQoS(),std::bind(&A300SafetyController::scanCb,this,std::placeholders::_1));
    desired_sub_=create_subscription<geometry_msgs::msg::Twist>(desired_topic_,10,std::bind(&A300SafetyController::cmdCb,this,std::placeholders::_1));
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,10,std::bind(&A300SafetyController::odomCb,this,std::placeholders::_1));
    safe_pub_=create_publisher<geometry_msgs::msg::Twist>(safe_topic_,10);
    state_pub_=create_publisher<std_msgs::msg::String>("a300/obstacle_state",10);
    timer_=create_wall_timer(std::chrono::milliseconds(20),std::bind(&A300SafetyController::loop,this));
    last_scan_=now();
  }
private:
  static double rad(double d){return d*M_PI/180.0;}
  double sector(const sensor_msgs::msg::LaserScan &s,double a,double b) const {
    double rmin=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<s.ranges.size();++i){
      double x=s.angle_min+i*s.angle_increment;
      if(x<a||x>b) continue;
      double r=s.ranges[i];
      if(std::isfinite(r)&&r>=s.range_min&&r<=s.range_max) rmin=std::min(rmin,r);
    }
    return rmin;
  }
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr s){
    std::lock_guard<std::mutex> l(scan_m_);
    front_=sector(*s,-rad(front_half_),rad(front_half_));
    fl_=sector(*s,rad(front_half_),rad(side_max_));
    fr_=sector(*s,-rad(side_max_),-rad(front_half_));
    left_=sector(*s,rad(side_min_),rad(side_max_));
    right_=sector(*s,-rad(side_max_),-rad(side_min_));
    last_scan_=now();
  }
  void cmdCb(const geometry_msgs::msg::Twist::SharedPtr c){
    std::lock_guard<std::mutex> l(cmd_m_); desired_=*c;
  }
  static double yawFromQuat(const nav_msgs::msg::Odometry &o){
    double qw=o.pose.pose.orientation.w, qx=o.pose.pose.orientation.x,
           qy=o.pose.pose.orientation.y, qz=o.pose.pose.orientation.z;
    double siny=2.0*(qw*qz+qx*qy), cosy=1.0-2.0*(qy*qy+qz*qz);
    return std::atan2(siny,cosy);
  }
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr o){
    std::lock_guard<std::mutex> l(scan_m_); yaw_=yawFromQuat(*o);
  }
  static double normPi(double a){
    while(a>M_PI) a-=2*M_PI;
    while(a<-M_PI) a+=2*M_PI;
    return a;
  }
  void beginAvoid(double yaw){
    if(avoid_phase_!=1){
      avoid_phase_=1;
      double diff = fr_-fl_;
      avoid_dir_ = (std::fabs(diff)<avoid_sym_eps_) ? avoid_default_dir_
                 : (diff>0.0 ? -1.0 : 1.0);
      start_yaw_ = std::isfinite(yaw) ? yaw : 0.0;
      avoid_target_ = start_yaw_ + avoid_dir_*(M_PI/2.0);  // turn ~90 deg to clear
    }
  }
  void steerToHeading(double yaw, geometry_msgs::msg::Twist &out){
    double d=normPi(yaw-avoid_target_);
    rclcpp::Time tnow=now();
    double dt=(have_yaw_time_) ? (tnow-last_yaw_time_).seconds() : 0.0;
    if(dt<0.0||dt>0.5) dt=0.0;
    double yaw_rate = (dt>1e-3) ? normPi(yaw-prev_yaw_)/dt : 0.0;
    prev_yaw_=yaw; last_yaw_time_=tnow; have_yaw_time_=true;
    double steer=std::clamp(-d*recover_gain_ - yaw_rate*recover_damp_, -wmax_, wmax_);
    out.angular.z += steer;
  }
  void loop(){
    geometry_msgs::msg::Twist out; {std::lock_guard<std::mutex> l(cmd_m_);out=desired_;}
    double f,l,r,fl,fr; rclcpp::Time t;
    {std::lock_guard<std::mutex> x(scan_m_);f=front_;l=left_;r=right_;fl=fl_;fr=fr_;t=last_scan_;}
    out.linear.x=std::clamp(out.linear.x,-vmax_,vmax_);
    out.angular.z=std::clamp(out.angular.z,-wmax_,wmax_);
    std::string state="SAFE";
    if((now()-t).seconds()>0.30){
      if(out.linear.x>0) out.linear.x=0;
      state="SCAN_STALE";
    } else if(out.linear.x>0){
      bool avoiding=false;
      // read current yaw (may be NaN if no odom yet)
      double yaw;
      {std::lock_guard<std::mutex> x(scan_m_);yaw=yaw_;}
      // respect user steering: if the user is actively turning, drop any
      // recovery and hand control back to them
      if(avoid_phase_!=0 && std::fabs(desired_.angular.z)>0.05){
        avoid_phase_=0; avoid_dir_=0.0;
      }
      if(f<=stop_){
        // RED zone: turn to the avoidance heading and keep moving; stop only
        // when neither side is open enough to continue.
        bool openR = fr>stop_;
        bool openL = fl>stop_;
        if(openR || openL){
          double open = std::max(fr, fl);
          out.linear.x *= std::clamp(open/slow_, 0.0, 1.0);
          out.linear.x = std::max(out.linear.x, avoid_min_speed_);
          beginAvoid(yaw);
          steerToHeading(yaw, out);
          avoiding=true;
        } else {
          out.linear.x=0; state="STOP";
        }
      } else if(f<=slow_){
        // ORANGE zone: slow down and steer toward the avoidance heading.
        double k=std::clamp((f-stop_)/(slow_-stop_),0.0,1.0);
        out.linear.x*=std::max(k,0.05);
        beginAvoid(yaw);
        steerToHeading(yaw, out);
        avoiding=true;
      } else if((avoid_phase_==1||avoid_phase_==2) && std::isfinite(yaw)){
        // RECOVERY zone (front opened past slow_, i.e. WARNING band or clear):
        // once the robot has cleared the wall (turned to the avoidance
        // heading and front is open), steer back to the heading the user had
        // when avoidance started, while keeping the user's forward speed.
        if(avoid_phase_==1 && std::fabs(normPi(yaw-avoid_target_)) < rad(20.0)){
          // reached the avoidance heading and front is clear: recover.
          avoid_phase_=2; avoid_target_=start_yaw_;
        }
        double d=normPi(yaw-avoid_target_);
        rclcpp::Time tnow=now();
        double dt=(have_yaw_time_) ? (tnow-last_yaw_time_).seconds() : 0.0;
        if(dt<0.0||dt>0.5) dt=0.0;   // guard stale/clamped clock
        double yaw_rate = (dt>1e-3) ? normPi(yaw-prev_yaw_)/dt : 0.0;
        prev_yaw_=yaw; last_yaw_time_=tnow; have_yaw_time_=true;
        if(std::fabs(d) < rad(recover_eps_deg_) && std::fabs(yaw_rate) < rad(3.0)){
          avoid_phase_=0; avoid_dir_=0.0;   // done
        } else {
          double steer=std::clamp(-d*recover_gain_ - yaw_rate*recover_damp_, -wmax_, wmax_);
          out.angular.z += steer;
          avoiding=true;
        }
        if(f<=warning_) state="WARNING";
      } else if(f<=warning_){
        // WARNING zone: gentle slowdown only.
        double k=std::clamp((f-slow_)/(warning_-slow_),0.0,1.0);
        out.linear.x*=(0.5+0.5*k);
        state="WARNING";
        // obstacle cleared since entering avoid: begin recovery toward the
        // heading the user had when avoidance started
        if(avoid_phase_==1){
          avoid_phase_=2; avoid_target_=start_yaw_;
        }
      } else {
        // front fully clear and not recovering: nothing to do.
      }
      if(avoiding) state="AVOIDING";
      if(out.angular.z>0&&l<stop_)out.angular.z=0;
      if(out.angular.z<0&&r<stop_)out.angular.z=0;
    } else if(f<=warning_){
      // moving backward / stopped: nothing to avoid
      if(avoid_phase_!=0) avoid_phase_=0; avoid_dir_=0.0;
    }
    if(!reverse_&&out.linear.x<0)out.linear.x=0;
    out.angular.z=std::clamp(out.angular.z,-wmax_,wmax_);
    RCLCPP_INFO(get_logger(), "front=%.2f fl=%.2f fr=%.2f left=%.2f right=%.2f state=%s linx=%.2f angz=%.2f phase=%d yaw=%.2f", f, fl, fr, l, r, state.c_str(), out.linear.x, out.angular.z, avoid_phase_, (std::isfinite(yaw_)?yaw_:0.0));
    out.linear.x = -out.linear.x;
    safe_pub_->publish(out);
    std_msgs::msg::String m;m.data=state;state_pub_->publish(m);
  }
  std::string scan_topic_,desired_topic_,safe_topic_,odom_topic_; double warning_,slow_,stop_,front_half_,side_min_,side_max_,vmax_,wmax_; bool reverse_; double avoid_gain_,avoid_min_speed_,avoid_default_dir_,avoid_sym_eps_,recover_gain_,recover_damp_,recover_eps_deg_,avoid_dir_;
  double front_{std::numeric_limits<double>::infinity()},fl_{},fr_{},left_{std::numeric_limits<double>::infinity()},right_{std::numeric_limits<double>::infinity()};
  double yaw_{std::numeric_limits<double>::quiet_NaN()},start_yaw_{0.0},avoid_target_{0.0}; int avoid_phase_{0};
  double prev_yaw_{0.0}; rclcpp::Time last_yaw_time_; bool have_yaw_time_{false};
  rclcpp::Time last_scan_; geometry_msgs::msg::Twist desired_; std::mutex scan_m_,cmd_m_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<A300SafetyController>());rclcpp::shutdown();return 0;}
