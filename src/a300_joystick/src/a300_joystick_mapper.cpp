#include <cmath>
#include <functional>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class A300JoystickMapper : public rclcpp::Node {
public:
  A300JoystickMapper() : Node("a300_joystick_mapper") {
    axis_linear_ = declare_parameter("axis_linear", 1);
    axis_angular_ = declare_parameter("axis_angular", 0);
    vmax_ = declare_parameter("max_linear_speed", 0.8);
    wmax_ = declare_parameter("max_angular_speed", 1.0);
    deadzone_ = declare_parameter("deadzone", 0.08);
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&A300JoystickMapper::cb, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_desired", 10);
  }
private:
  double dz(double x) const {
    if (std::abs(x) < deadzone_) return 0.0;
    return (x > 0.0 ? 1.0 : -1.0) *
      (std::abs(x) - deadzone_) / (1.0 - deadzone_);
  }
  void cb(const sensor_msgs::msg::Joy::SharedPtr m) {
    if (axis_linear_ >= static_cast<int>(m->axes.size()) ||
        axis_angular_ >= static_cast<int>(m->axes.size())) return;
    geometry_msgs::msg::Twist c;
    c.linear.x = dz(m->axes[axis_linear_]) * vmax_;
    c.angular.z = dz(m->axes[axis_angular_]) * wmax_;
    pub_->publish(c);
  }
  int axis_linear_, axis_angular_;
  double vmax_, wmax_, deadzone_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A300JoystickMapper>());
  rclcpp::shutdown();
  return 0;
}