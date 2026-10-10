```cpp
#include "meu_pid_controller/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace meu_pid_controller
{

PIDController::PIDController()
: active_(false),
  integral_linear_(0.0),
  integral_angular_(0.0)
{
}

void PIDController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent_.lock();

  if (!node) {
    throw std::runtime_error(
      "Não foi possível acessar o LifecycleNode.");
  }

  // ============================================================
  // PARÂMETROS PID
  // ============================================================

  kp_linear_ = node->declare_parameter<double>(
    plugin_name_ + ".kp_linear", 0.8);

  ki_linear_ = node->declare_parameter<double>(
    plugin_name_ + ".ki_linear", 0.0);

  kd_linear_ = node->declare_parameter<double>(
    plugin_name_ + ".kd_linear", 0.0);

  kp_angular_ = node->declare_parameter<double>(
    plugin_name_ + ".kp_angular", 1.5);

  ki_angular_ = node->declare_parameter<double>(
    plugin_name_ + ".ki_angular", 0.0);

  kd_angular_ = node->declare_parameter<double>(
    plugin_name_ + ".kd_angular", 0.0);

  // ============================================================
  // LIMITES
  // ============================================================

  max_linear_velocity_ = node->declare_parameter<double>(
    plugin_name_ + ".max_linear_velocity", 0.10);

  max_angular_velocity_ = node->declare_parameter<double>(
    plugin_name_ + ".max_angular_velocity", 0.40);

  // ============================================================
  // ESTADO
  // ============================================================

  integral_linear_ = 0.0;
  integral_angular_ = 0.0;

  last_time_ = node->now();

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] PID Controller configurado.",
    plugin_name_.c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] Linear: Kp=%.3f Ki=%.3f Kd=%.3f",
    plugin_name_.c_str(),
    kp_linear_,
    ki_linear_,
    kd_linear_);

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] Angular: Kp=%.3f Ki=%.3f Kd=%.3f",
    plugin_name_.c_str(),
    kp_angular_,
    ki_angular_,
    kd_angular_);

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] Limites: v=%.3f m/s, w=%.3f rad/s",
    plugin_name_.c_str(),
    max_linear_velocity_,
    max_angular_velocity_);
}

void PIDController::cleanup()
{
  global_plan_.poses.clear();

  integral_linear_ = 0.0;
  integral_angular_ = 0.0;

  active_ = false;

  auto node = parent_.lock();

  if (node) {
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] PID Controller finalizado.",
      plugin_name_.c_str());
  }
}

void PIDController::activate()
{
  active_ = true;

  integral_linear_ = 0.0;
  integral_angular_ = 0.0;

  auto node = parent_.lock();

  if (node) {
    last_time_ = node->now();

    RCLCPP_INFO(
      node->get_logger(),
      "[%s] PID Controller ativado.",
      plugin_name_.c_str());
  }
}

void PIDController::deactivate()
{
  active_ = false;

  integral_linear_ = 0.0;
  integral_angular_ = 0.0;

  auto node = parent_.lock();

  if (node) {
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] PID Controller desativado.",
      plugin_name_.c_str());
  }
}

void PIDController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (percentage) {
    max_linear_velocity_ *= speed_limit / 100.0;
  } else {
    max_linear_velocity_ = speed_limit;
  }
}

void PIDController::setPlan(
  const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  auto node = parent_.lock();

  if (node) {
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] Novo plano recebido: %zu poses | frame: %s",
      plugin_name_.c_str(),
      global_plan_.poses.size(),
      global_plan_.header.frame_id.c_str());
  }
}

geometry_msgs::msg::TwistStamped
PIDController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd;

  auto node = parent_.lock();

  if (!node) {
    return cmd;
  }

  // ============================================================
  // COMANDO SEGURO INICIAL
  // ============================================================

  cmd.header.stamp = node->now();
  cmd.header.frame_id = "base_link";

  cmd.twist.linear.x = 0.0;
  cmd.twist.linear.y = 0.0;
  cmd.twist.linear.z = 0.0;

  cmd.twist.angular.x = 0.0;
  cmd.twist.angular.y = 0.0;
  cmd.twist.angular.z = 0.0;

  if (!active_ || global_plan_.poses.empty()) {
    return cmd;
  }

  // ============================================================
  // 1. TRANSFORMAR POSE ATUAL PARA O FRAME DO PLANO
  // ============================================================

  geometry_msgs::msg::PoseStamped current_pose_plan;

  try {
    current_pose_plan = tf_->transform(
      pose,
      global_plan_.header.frame_id,
      tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException & ex) {

    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      2000,
      "[%s] Falha ao transformar pose atual: %s",
      plugin_name_.c_str(),
      ex.what());

    return cmd;
  }

  // ============================================================
  // 2. ENCONTRAR PONTO MAIS PRÓXIMO
  // ============================================================

  size_t nearest_index = 0;

  double nearest_distance =
    std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {

    const double dx =
      global_plan_.poses[i].pose.position.x -
      current_pose_plan.pose.position.x;

    const double dy =
      global_plan_.poses[i].pose.position.y -
      current_pose_plan.pose.position.y;

    const double distance =
      std::hypot(dx, dy);

    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  // ============================================================
  // 3. LOOKAHEAD
  // ============================================================

  constexpr double lookahead_distance = 0.25;

  size_t target_index = nearest_index;

  for (
    size_t i = nearest_index;
    i < global_plan_.poses.size();
    ++i)
  {
    const double dx =
      global_plan_.poses[i].pose.position.x -
      current_pose_plan.pose.position.x;

    const double dy =
      global_plan_.poses[i].pose.position.y -
      current_pose_plan.pose.position.y;

    const double distance =
      std::hypot(dx, dy);

    if (distance >= lookahead_distance) {
      target_index = i;
      break;
    }

    target_index = i;
  }

  // ============================================================
  // 4. OBJETIVO FINAL
  // ============================================================

  const auto & final_pose =
    global_plan_.poses.back();

  const double final_dx =
    final_pose.pose.position.x -
    current_pose_plan.pose.position.x;

  const double final_dy =
    final_pose.pose.position.y -
    current_pose_plan.pose.position.y;

  const double final_distance =
    std::hypot(final_dx, final_dy);

  // ============================================================
  // 5. TRANSFORMAR LOOKAHEAD PARA BASE_LINK
  // ============================================================

  geometry_msgs::msg::PoseStamped target_pose_base;

  try {
    target_pose_base = tf_->transform(
      global_plan_.poses[target_index],
      pose.header.frame_id,
      tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException & ex) {

    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      2000,
      "[%s] Falha ao transformar alvo: %s",
      plugin_name_.c_str(),
      ex.what());

    return cmd;
  }

  // ============================================================
  // 6. ERRO DE POSIÇÃO NO FRAME DO ROBÔ
  // ============================================================

  const double target_x =
    target_pose_base.pose.position.x;

  const double target_y =
    target_pose_base.pose.position.y;

  const double position_error =
    std::hypot(target_x, target_y);

  // ============================================================
  // 7. ERRO ANGULAR
  //
  // IMPORTANTE:
  //
  // atan2(y, x) segue a convenção ROS:
  //
  // +angular.z -> rotação anti-horária -> esquerda
  // -angular.z -> rotação horária      -> direita
  //
  // NÃO inverter o sinal aqui.
  // ============================================================

  double heading_error =
    std::atan2(target_y, target_x);

  // ============================================================
  // 8. ORIENTAÇÃO FINAL
  // ============================================================

  constexpr double final_position_tolerance = 0.10;
  constexpr double final_yaw_tolerance = 0.05;

  const bool final_orientation_mode =
    final_distance <= final_position_tolerance;

  double final_yaw_error = 0.0;

  if (final_orientation_mode) {

    tf2::Quaternion q_current;

    tf2::fromMsg(
      current_pose_plan.pose.orientation,
      q_current);

    tf2::Quaternion q_final;

    tf2::fromMsg(
      final_pose.pose.orientation,
      q_final);

    double roll_current;
    double pitch_current;
    double yaw_current;

    double roll_final;
    double pitch_final;
    double yaw_final;

    tf2::Matrix3x3(q_current).getRPY(
      roll_current,
      pitch_current,
      yaw_current);

    tf2::Matrix3x3(q_final).getRPY(
      roll_final,
      pitch_final,
      yaw_final);

    // ==========================================================
    // ERRO ANGULAR NORMALIZADO PARA [-PI, PI]
    // ==========================================================

    final_yaw_error =
      std::atan2(
        std::sin(yaw_final - yaw_current),
        std::cos(yaw_final - yaw_current));

    heading_error = final_yaw_error;
  }

  // ============================================================
  // 9. DT REAL
  // ============================================================

  const rclcpp::Time current_time =
    node->now();

  double dt =
    (current_time - last_time_).seconds();

  if (dt <= 0.0 || dt > 1.0) {
    dt = 0.02;
  }

  last_time_ = current_time;

  // ============================================================
  // 10. PID LINEAR
  // ============================================================

  double linear_error = final_distance;

  if (!final_orientation_mode) {
    linear_error = position_error;
  }

  integral_linear_ +=
    linear_error * dt;

  // Anti-windup
  integral_linear_ =
    std::clamp(
      integral_linear_,
      -0.5,
      0.5);

  // ============================================================
  // DERIVADA LINEAR
  //
  // Mantida em zero nesta etapa para evitar amplificação
  // do ruído da odometria.
  // ============================================================

  const double linear_derivative = 0.0;

  double linear_control =
    kp_linear_ * linear_error +
    ki_linear_ * integral_linear_ +
    kd_linear_ * linear_derivative;

  // ============================================================
  // 11. PID ANGULAR
  // ============================================================

  integral_angular_ +=
    heading_error * dt;

  // Anti-windup
  integral_angular_ =
    std::clamp(
      integral_angular_,
      -0.5,
      0.5);

  // ============================================================
  // DERIVADA ANGULAR
  //
  // Nesta primeira etapa permanece zero para evitar que o ruído
  // do heading produza oscilações.
  // ============================================================

  const double angular_derivative = 0.0;

  double angular_control =
    kp_angular_ * heading_error +
    ki_angular_ * integral_angular_ +
    kd_angular_ * angular_derivative;

  // ============================================================
  // 12. SATURAÇÃO
  // ============================================================

  linear_control =
    std::clamp(
      linear_control,
      0.0,
      max_linear_velocity_);

  angular_control =
    std::clamp(
      angular_control,
      -max_angular_velocity_,
      max_angular_velocity_);

  // ============================================================
  // 13. GIRAR ANTES DE ANDAR
  //
  // Se o alvo estiver muito fora da direção frontal,
  // primeiro alinhar o robô.
  // ============================================================

  constexpr double rotate_in_place_threshold = 0.30;

  if (
    !final_orientation_mode &&
    std::abs(heading_error) >
      rotate_in_place_threshold)
  {
    linear_control = 0.0;
  }

  // ============================================================
  // 14. CONTROLE DA ORIENTAÇÃO FINAL
  // ============================================================

  if (final_orientation_mode) {

    // Não avançar enquanto corrige orientação.
    linear_control = 0.0;

    if (
      std::abs(final_yaw_error) <=
      final_yaw_tolerance)
    {
      angular_control = 0.0;
    }
  }

  // ============================================================
  // 15. PUBLICAR COMANDO
  //
  // NÃO inverter angular_control.
  //
  // +angular.z = esquerda
  // -angular.z = direita
  // ============================================================

  cmd.twist.linear.x =
    linear_control;

  cmd.twist.angular.z =
    angular_control;

  // ============================================================
  // 16. LOG DE DEBUG
  // ============================================================

  RCLCPP_INFO_THROTTLE(
    node->get_logger(),
    *node->get_clock(),
    500,

    "[%s] alvo=%zu/%zu | "
    "dist_final=%.3f | "
    "erro_pos=%.3f | "
    "heading=%.3f | "
    "yaw_final=%.3f | "
    "modo_final=%s | "
    "v=%.3f | "
    "w=%.3f",

    plugin_name_.c_str(),

    target_index,
    global_plan_.poses.size(),

    final_distance,
    position_error,

    heading_error,
    final_yaw_error,

    final_orientation_mode
      ? "SIM"
      : "NAO",

    linear_control,
    angular_control);

  return cmd;
}

}  // namespace meu_pid_controller

PLUGINLIB_EXPORT_CLASS(
  meu_pid_controller::PIDController,
  nav2_core::Controller)
```
