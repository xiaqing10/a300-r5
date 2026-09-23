#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class A300SafetyController : public rclcpp::Node {
public:
  A300SafetyController():Node("a300_safety_controller") {
    scan_topic_=declare_parameter("scan_topic",std::string("/scan"));
    desired_topic_=declare_parameter("desired_cmd_topic",std::string("/cmd_vel_desired"));
    safe_topic_=declare_parameter("safe_cmd_topic",std::string("/cmd_vel"));
    warning_=declare_parameter("warning_distance",1.20);
    slow_=declare_parameter("slow_distance",0.80);
    stop_=declare_parameter("stop_distance",0.35);
    front_half_=declare_parameter("front_half_angle_deg",30.0);
    side_min_=declare_parameter("side_angle_min_deg",30.0);
    side_max_=declare_parameter("side_angle_max_deg",100.0);
    vmax_=declare_parameter("max_linear_speed",0.80);
    wmax_=declare_parameter("max_angular_speed",1.00);
    reverse_=declare_parameter("reverse_allowed",false);
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_,rclcpp::SensorDataQoS(),std::bind(&A300SafetyController::scanCb,this,std::placeholders::_1));
    desired_sub_=create_subscription<geometry_msgs::msg::Twist>(desired_topic_,10,std::bind(&A300SafetyController::cmdCb,this,std::placeholders::_1));
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
  void loop(){
    geometry_msgs::msg::Twist out; {std::lock_guard<std::mutex> l(cmd_m_);out=desired_;}
    double f,l,r; rclcpp::Time t;
    {std::lock_guard<std::mutex> x(scan_m_);f=front_;l=left_;r=right_;t=last_scan_;}
    out.linear.x=std::clamp(out.linear.x,-vmax_,vmax_);
    out.angular.z=std::clamp(out.angular.z,-wmax_,wmax_);
    std::string state="SAFE";
    if((now()-t).seconds()>0.30){
      if(out.linear.x>0) out.linear.x=0;
      state="SCAN_STALE";
    } else if(out.linear.x>0){
      if(f<=stop_){out.linear.x=0;state="STOP";}
      else if(f<=slow_){double k=std::clamp((f-stop_)/(slow_-stop_),0.0,1.0);out.linear.x*=std::max(k,0.05);state="SLOW";}
      else if(f<=warning_){double k=std::clamp((f-slow_)/(warning_-slow_),0.0,1.0);out.linear.x*=(0.5+0.5*k);state="WARNING";}
      if(out.angular.z>0&&l<stop_)out.angular.z=0;
      if(out.angular.z<0&&r<stop_)out.angular.z=0;
    }
    if(!reverse_&&out.linear.x<0)out.linear.x=0;
    RCLCPP_INFO(get_logger(), "front=%.2f left=%.2f right=%.2f state=%s linx=%.2f", f, l, r, state.c_str(), out.linear.x);
    out.linear.x = -out.linear.x;
    safe_pub_->publish(out);
    std_msgs::msg::String m;m.data=state;state_pub_->publish(m);
  }
  std::string scan_topic_,desired_topic_,safe_topic_; double warning_,slow_,stop_,front_half_,side_min_,side_max_,vmax_,wmax_; bool reverse_;
  double front_{std::numeric_limits<double>::infinity()},fl_{},fr_{},left_{std::numeric_limits<double>::infinity()},right_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_scan_; geometry_msgs::msg::Twist desired_; std::mutex scan_m_,cmd_m_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<A300SafetyController>());rclcpp::shutdown();return 0;}