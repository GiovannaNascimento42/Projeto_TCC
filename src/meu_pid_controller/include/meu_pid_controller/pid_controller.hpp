#ifndef MEU_PID_CONTROLLER__PID_CONTROLLER_HPP_
#define MEU_PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace meu_pid_controller
{

class PIDController : public nav2_core::Controller
{
public:

  PIDController();
  ~PIDController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

  void setPlan(
    const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string plugin_name_;

  nav_msgs::msg::Path global_plan_;

  double kp_linear_;
  double ki_linear_;
  double kd_linear_;

  double kp_angular_;
  double ki_angular_;
  double kd_angular_;

  double max_linear_velocity_;
  double max_angular_velocity_;

  double integral_linear_;
  double integral_angular_;

  rclcpp::Time last_time_;

  bool active_{false};
};

}  // namespace meu_pid_controller

#endif  // MEU_PID_CONTROLLER__PID_CONTROLLER_HPP_
