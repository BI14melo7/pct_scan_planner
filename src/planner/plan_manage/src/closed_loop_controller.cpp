#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 1.0);
    heading_lookahead_distance_ =
        declare_parameter<double>("heading_lookahead_distance", 0.45);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 1.05);
    heading_hold_threshold_ = declare_parameter<double>("heading_hold_threshold", 0.18);
    large_turn_confirm_time_ =
        declare_parameter<double>("large_turn_confirm_time", 0.35);
    side_step_distance_ = declare_parameter<double>("side_step_distance", 1.2);
    side_step_min_angle_ = declare_parameter<double>("side_step_min_angle", 0.22);
    side_step_max_angle_ = declare_parameter<double>("side_step_max_angle", 0.65);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.25);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    // Use a spatial chord instead of the instantaneous B-spline tangent.
    // Replans can preserve a stale initial derivative for a few centimetres;
    // following that tangent directly makes the body turn backwards and then
    // immediately turn forward again.  Looking farther along the same path
    // ignores that small transient without changing the translation path.
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    const double t_end = std::min(traj_duration_, t_cur + time_forward_);
    const double sample_dt = std::max(0.05, std::min(0.15, time_forward_ / 6.0));
    for (double t = std::min(t_cur + sample_dt, t_end);
         t <= t_end + 1e-6; t += sample_dt)
    {
      const Eigen::Vector3d candidate =
          traj_[0].evaluateDeBoorT(std::min(t, t_end)) - pos_des;
      if (candidate.head<2>().squaredNorm() > direction.head<2>().squaredNorm())
        direction = candidate;
      if (candidate.head<2>().norm() >= heading_lookahead_distance_)
        break;
    }
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[0].evaluateDeBoorT(traj_duration_) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;

    // A short local detour with a moderate heading change is better handled
    // as a quadruped side-step.  Larger or longer turns still rotate the body
    // toward the path.  The 37 degree upper bound also keeps the difference
    // between the path-aligned and body-aligned collision footprints inside
    // the planner's existing execution-clearance envelope.  Select the mode
    // once per trajectory so a tangent near a threshold cannot cause chatter.
    side_step_active_ = false;
    if (have_odom_)
    {
      const Eigen::Vector3d start_pos = traj_[0].evaluateDeBoorT(0.0);
      const Eigen::Vector3d terminal_pos = traj_[0].evaluateDeBoorT(traj_duration_);
      const double terminal_distance =
          (terminal_pos.head<2>() - odom_pos_.head<2>()).norm();
      const double initial_yaw_error =
          normalizeAngle(estimateDesiredYaw(0.0, start_pos) - odom_yaw_);
      const double abs_initial_yaw_error = std::abs(initial_yaw_error);
      side_step_active_ =
          terminal_distance <= side_step_distance_ &&
          abs_initial_yaw_error >= side_step_min_angle_ &&
          abs_initial_yaw_error <= side_step_max_angle_;
      if (side_step_active_)
        RCLCPP_INFO(get_logger(),
                    "Short detour %.2f m at %.1f deg: locking lateral side-step for trajectory %lld",
                    terminal_distance, initial_yaw_error * 180.0 / M_PI,
                    static_cast<long long>(traj_id_));
      else if (abs_initial_yaw_error > 2.1)
        RCLCPP_WARN(get_logger(),
                    "Trajectory %lld requests a %.1f deg initial turn; waiting for stable heading before turning",
                    static_cast<long long>(traj_id_),
                    initial_yaw_error * 180.0 / M_PI);
    }
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double path_yaw_error =
        normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double abs_path_yaw_error = std::abs(path_yaw_error);

    // A locked side-step keeps the current body heading for the whole local
    // trajectory.  Tiny heading errors are ignored in either mode.
    const bool hold_heading =
        side_step_active_ || abs_path_yaw_error <= heading_hold_threshold_;

    const double yaw_error = hold_heading ? 0.0 : path_yaw_error;
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (!hold_heading && abs_path_yaw_error > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      const double desired_yaw = normalizeAngle(odom_yaw_ + path_yaw_error);
      if (!large_turn_pending_ ||
          std::abs(normalizeAngle(desired_yaw - pending_turn_yaw_)) > 0.35)
      {
        large_turn_pending_ = true;
        pending_turn_yaw_ = desired_yaw;
        large_turn_pending_since_ = current_time;
      }

      const double pending_time =
          (current_time - large_turn_pending_since_).seconds();
      if (pending_time >= large_turn_confirm_time_)
        publishStop(yaw_command);
      else
        publishStop();
      last_update_time_ = current_time;
      return;
    }

    large_turn_pending_ = false;

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    Eigen::Vector2d vel_body(c * vel_world.x() + s * vel_world.y(),
                            -s * vel_world.x() + c * vel_world.y());
    // Preserve the requested travel direction when the slower lateral axis
    // saturates; independent clipping would bend a side-step back toward the
    // robot's forward axis.
    double body_scale = 1.0;
    if (std::abs(vel_body.x()) > max_vx_)
      body_scale = std::min(body_scale, max_vx_ / std::abs(vel_body.x()));
    if (std::abs(vel_body.y()) > max_vy_)
      body_scale = std::min(body_scale, max_vy_ / std::abs(vel_body.y()));
    vel_body *= body_scale;

    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(vel_body.x(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(vel_body.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool side_step_active_{false};
  bool large_turn_pending_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time large_turn_pending_since_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_lookahead_distance_;
  double heading_error_threshold_, heading_hold_threshold_;
  double large_turn_confirm_time_;
  double pending_turn_yaw_{0.0};
  double side_step_distance_, side_step_min_angle_, side_step_max_angle_;
  double kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
