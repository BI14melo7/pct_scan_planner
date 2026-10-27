
#include <plan_manage/scan_replan_fsm.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    replan_failure_active_ = false;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    goal_tolerance_ = std::max(
        0.05, load_parameter<double>(node_, "fsm.goal_tolerance", 0.5));
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 20);
    replan_failure_grace_ = std::max(
        0.0, load_parameter<double>(node_, "fsm.replan_failure_grace", 6.0));
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    rejected_target_radius_ = std::max(
        0.05, load_parameter<double>(node_, "fsm.rejected_target_radius", 0.5));
    local_target_clearance_ = std::max(
        0.0, load_parameter<double>(node_, "fsm.local_target_clearance", 0.25));
    const double rolling_corner_stop_angle_deg = std::clamp(
        load_parameter<double>(node_, "fsm.rolling_corner_stop_angle_deg", 120.0),
        30.0, 179.0);
    rolling_corner_stop_angle_ = rolling_corner_stop_angle_deg * M_PI / 180.0;
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
      cancelActiveTarget(true, "Preset navigation aborted: first waypoint is not safely reachable");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    Eigen::Vector3d requested_target(
        msg->poses[0].pose.position.x,
        msg->poses[0].pose.position.y,
        rviz_goal_height_);
    if (isPreviouslyRejectedTarget(requested_target))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject navigation goal [%.2f, %.2f, %.2f]: this no-go target was rejected earlier",
                   requested_target(0), requested_target(1), requested_target(2));
      return;
    }
    if (isTargetKnownOccupied(requested_target))
    {
      rememberRejectedTarget(requested_target);
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject navigation goal [%.2f, %.2f, %.2f]: target footprint overlaps an obstacle",
                   requested_target(0), requested_target(1), requested_target(2));
      return;
    }
    if ((requested_target - odom_pos_).norm() < 0.05)
    {
      RCLCPP_INFO(node_->get_logger(), "Navigation goal is already reached; waiting for another goal");
      return;
    }

    bool success = planner_manager_->planGlobalTraj(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), requested_target,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = requested_target;
      success = validateGlobalTarget();
    }

    if (success)
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      cout << "Triggered!" << endl;
      trigger_ = true;
      init_pt_ = odom_pos_;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      else if (exec_state_ == EMERGENCY_STOP)
        RCLCPP_INFO(node_->get_logger(), "Valid replacement goal queued while stopping");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      rememberRejectedTarget(requested_target);
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    if (isPreviouslyRejectedTarget(waypoints.back()))
    {
      const auto &target = waypoints.back();
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject reference path: terminal [%.2f, %.2f, %.2f] is a remembered no-go target",
                   target(0), target(1), target(2));
      return false;
    }

    if (isTargetKnownOccupied(waypoints.back()))
    {
      const auto &target = waypoints.back();
      rememberRejectedTarget(target);
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject reference path: terminal footprint [%.2f, %.2f, %.2f] overlaps an obstacle",
                   target(0), target(1), target(2));
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      rememberRejectedTarget(waypoints.back());
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!validateGlobalTarget())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    const Eigen::Vector3d requested_target = active_waypoints_[current_wp_];
    if (isPreviouslyRejectedTarget(requested_target))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject waypoint %d: target [%.2f, %.2f, %.2f] is a remembered no-go target",
                   current_wp_ + 1, requested_target(0), requested_target(1), requested_target(2));
      return false;
    }
    if (isTargetKnownOccupied(requested_target))
    {
      rememberRejectedTarget(requested_target);
      RCLCPP_ERROR(node_->get_logger(),
                   "Reject waypoint %d: target footprint [%.2f, %.2f, %.2f] overlaps an obstacle",
                   current_wp_ + 1, requested_target(0), requested_target(1), requested_target(2));
      return false;
    }

    end_pt_ = requested_target;
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      rememberRejectedTarget(requested_target);
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!validateGlobalTarget())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::isPreviouslyRejectedTarget(const Eigen::Vector3d &target) const
  {
    const double radius_squared = rejected_target_radius_ * rejected_target_radius_;
    return std::any_of(
        rejected_targets_.begin(), rejected_targets_.end(),
        [&](const Eigen::Vector3d &rejected) {
          return (rejected - target).squaredNorm() <= radius_squared;
        });
  }

  void SCANReplanFSM::rememberRejectedTarget(const Eigen::Vector3d &target)
  {
    if (isPreviouslyRejectedTarget(target))
      return;

    rejected_targets_.push_back(target);
    RCLCPP_WARN(node_->get_logger(),
                "Remember no-go target [%.2f, %.2f, %.2f] with %.2f m rejection radius",
                target(0), target(1), target(2), rejected_target_radius_);
  }

  bool SCANReplanFSM::isTargetKnownOccupied(const Eigen::Vector3d &target) const
  {
    const auto map = planner_manager_->grid_map_;
    if (!map)
      return false;

    // A negative value means the target is outside the current sliding local
    // map.  It may still be a valid distant global target and will be checked
    // continuously as the map moves toward it.  Only a positively observed,
    // inflated obstacle is rejected here.
    const double yaw = estimateYawFromSegment(odom_pos_, target);
    return map->getInflateOccupancyWithClearance(
               target, yaw, planner_manager_->pp_.execution_clearance_) > 0;
  }

  bool SCANReplanFSM::validateGlobalTarget()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancyWithClearance(
        final_pt, estimateYawFromSegment(final_prev, final_pt),
        planner_manager_->pp_.execution_clearance_);
    if (final_occ <= 0)
      return true;

    RCLCPP_ERROR(node_->get_logger(),
                 "Reject target [%.2f, %.2f, %.2f]: terminal footprint overlaps an obstacle",
                 end_pt_(0), end_pt_(1), end_pt_(2));
    rememberRejectedTarget(end_pt_);
    return false;
  }

  void SCANReplanFSM::cancelActiveTarget(bool emergency_stop, const char *reason)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", reason);
    clearReplanFailures();
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    active_waypoints_.clear();
    current_wp_ = 0;

    if (emergency_stop)
    {
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "TARGET_SAFETY");
    }
    else
    {
      changeFSMExecState(WAIT_TARGET, "TARGET_SAFETY");
    }
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      waypoints.push_back(wp);
    }
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      trigger_ = true;
      init_pt_ = odom_pos_;
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      else if (exec_state_ == EMERGENCY_STOP)
      {
        RCLCPP_INFO(node_->get_logger(), "Valid replacement path queued while stopping");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        clearReplanFailures();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        noteReplanFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        clearReplanFailures();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        noteReplanFailure();
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < goal_tolerance_)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        cancelActiveTarget(true, "Waypoint sequence aborted: next waypoint is invalid or unreachable");
        return;
      }

      // Finishing a receding-horizon B-spline is not the same as reaching the
      // navigation goal.  With a short local horizon, one global route is
      // intentionally executed as several local trajectories.
      if (t_cur > info->duration_ - 1e-2)
      {
        const bool reached_global_target =
            (end_pt_ - odom_pos_).norm() < goal_tolerance_;

        if (!reached_global_target)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Local segment complete; %.2f m remain to the global target, continuing",
                      (end_pt_ - odom_pos_).norm());
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          cancelActiveTarget(true, "Waypoint sequence aborted: next waypoint is invalid or unreachable");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // Once the robot is inside the navigation-goal tolerance, keep executing
      // the already validated terminal trajectory.  Asking the planner for a
      // new sub-20 cm spline would be reported as "Close to goal" and could
      // otherwise be miscounted as a planning failure.
      else if ((end_pt_ - odom_pos_).norm() < goal_tolerance_)
      {
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          need_hover_stop_ = false;
          if (have_target_)
          {
            RCLCPP_INFO(node_->get_logger(),
                        "Emergency stop complete; starting the queued replacement target");
            trigger_ = true;
            have_new_target_ = true;
            changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_EXIT");
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(),
                        "Emergency stop complete; waiting for a new target");
            trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
          }
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ < max_replan_fail_count_ || !replan_failure_active_)
      return;

    const double failure_duration = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - replan_failure_start_).count();
    if (failure_duration < replan_failure_grace_)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Planning has failed %d times over %.1fs; keeping the target during the %.1fs recovery window",
          replan_fail_count_, failure_duration, replan_failure_grace_);
      return;
    }

    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      if (isTargetKnownOccupied(end_pt_))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Collision-free planning failed %d consecutive times and the "
                     "target footprint is occupied; remembering it as no-go",
                     replan_fail_count_);
        rememberRejectedTarget(end_pt_);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Collision-free planning failed %d consecutive times, but the "
                     "target footprint is free; canceling without marking it no-go",
                     replan_fail_count_);
      }
      cancelActiveTarget(
          true,
          "Active target canceled; waiting for a different navigation target");
    }
  }

  void SCANReplanFSM::noteReplanFailure()
  {
    if (!replan_failure_active_)
    {
      replan_failure_active_ = true;
      replan_failure_start_ = std::chrono::steady_clock::now();
    }
    ++replan_fail_count_;
  }

  void SCANReplanFSM::clearReplanFailures()
  {
    replan_fail_count_ = 0;
    replan_failure_active_ = false;
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!validateGlobalTarget())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    if (have_target_ && isTargetKnownOccupied(end_pt_))
    {
      rememberRejectedTarget(end_pt_);
      cancelActiveTarget(
          true,
          "Active target has become occupied; canceling it and waiting for a new goal");
      return;
    }

    // The closed-loop controller freezes trajectory time while the robot is
    // rotating in place toward a large turn.  Rechecking the path-aligned
    // footprint during that stationary rotation used to publish a replacement
    // trajectory at 20 Hz, repeatedly changing the requested heading before
    // the turn could finish.  No translation occurs while frozen, so defer
    // trajectory collision replanning until body alignment is complete.
    if (go2_execution_frozen_)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = std::max(0.0, (node_->now() - info->start_time_).seconds());
    const double initial_t = std::min(t_cur, info->duration_);
    const Eigen::Vector3d clearance_start =
        info->position_traj_.evaluateDeBoorT(initial_t);
    const Eigen::Vector3d clearance_start_prev =
        info->position_traj_.evaluateDeBoorT(std::max(initial_t - time_step, 0.0));
    const Eigen::Vector3d clearance_start_next = info->position_traj_.evaluateDeBoorT(
        std::min(initial_t + time_step, info->duration_));
    const double clearance_start_yaw = estimateYawFromSegment(
        clearance_start_prev, clearance_start_next);
    const bool recovering_clearance =
        map->getInflateOccupancy(clearance_start, clearance_start_yaw) == 0 &&
        map->getInflateOccupancyWithClearance(
            clearance_start, clearance_start_yaw,
            planner_manager_->pp_.execution_clearance_) != 0;

    for (double t = t_cur; t <= info->duration_ + 1e-6; t += time_step)
    {
      const double tc = std::min(t, info->duration_);
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(tc);
      Eigen::Vector3d pos_prev = info->position_traj_.evaluateDeBoorT(std::max(tc - time_step, 0.0));
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(tc + time_step, info->duration_));
      double required_clearance = planner_manager_->pp_.execution_clearance_;
      if (recovering_clearance && tc < info->duration_ - 1e-6)
      {
        const double displacement =
            (pos.head<2>() - clearance_start.head<2>()).norm();
        const double recovery_ratio = std::min(
            1.0, displacement /
                     planner_manager_->pp_.clearance_recovery_distance_);
        required_clearance *= recovery_ratio;
      }
      if (map->getInflateOccupancyWithClearance(
              pos, estimateYawFromSegment(pos_prev, pos_next),
              required_clearance) != 0)
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          // Never keep executing a trajectory known to collide while retrying
          // in the background.  Publish a stop first, then resume replanning
          // toward the still-valid target after odometry reports zero speed.
          RCLCPP_WARN(node_->get_logger(),
                      "Unsafe trajectory sample in %.3fs; replan failed, stopping before retry",
                      std::max(0.0, t - t_cur));
          need_hover_stop_ = false;
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          return;
        }
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    // Manual/preset goals do not come with an external global path.  Rebuild
    // their global reference against the current rolling occupancy map before
    // selecting a local target; otherwise the reference is only a straight
    // minimum-distance polynomial and can place the local goal behind a wall.
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH)
    {
      if (!planner_manager_->planGlobalTrajWithLocalDetour(
              start_pt_, start_vel_, start_acc_, end_pt_,
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "No obstacle-aware global route is currently available");
        return false;
      }

      // Display the collision-free rolling polyline itself; the smoothed
      // polynomial may cut corners and hide the actual global detour.
      visualization_->displayGlobalPathList(
          planner_manager_->global_route_points_, 0.1, 0);
    }

    if (!getLocalTarget())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No collision-free local target is available; refusing to publish a trajectory");
      return false;
    }

    // A replan normally inherits velocity from the previous B-spline.  Near
    // an obstacle corner that derivative can point away from the newly chosen
    // A-star segment, creating a short backwards lobe.  The controller then
    // sees a near-180 degree tangent, turns around, and turns back once the
    // spline resumes forward progress.  Remove only the incompatible XY
    // component while keeping a forward component when one exists.
    Eigen::Vector2d local_delta =
        local_target_pt_.head<2>() - start_pt_.head<2>();
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&
        planner_manager_->global_route_points_.size() >= 2)
    {
      const Eigen::Vector2d route_start_delta =
          planner_manager_->global_route_points_[1].head<2>() -
          start_pt_.head<2>();
      if (route_start_delta.norm() > 1e-3)
        local_delta = route_start_delta;
    }
    const double local_distance = local_delta.norm();
    const double start_speed_xy = start_vel_.head<2>().norm();
    if (local_distance > 1e-3 && start_speed_xy > 0.05)
    {
      const Eigen::Vector2d local_direction = local_delta / local_distance;
      const double forward_speed = start_vel_.head<2>().dot(local_direction);
      const double heading_cosine = forward_speed / start_speed_xy;
      if (heading_cosine < 0.5)
      {
        start_vel_.head<2>() =
            local_direction * std::max(0.0, forward_speed);
        start_acc_.head<2>().setZero();
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Discarding replan start velocity %.1f deg away from the local route",
            std::acos(std::clamp(heading_cosine, -1.0, 1.0)) * 180.0 / M_PI);
      }
    }

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  bool SCANReplanFSM::getLocalTarget()
  {
    const double t_step = std::max(
        0.01, planning_horizon_ / 20.0 / std::max(0.1, planner_manager_->pp_.max_vel_));
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    const auto &rolling_route = planner_manager_->global_route_points_;
    const bool use_rolling_route =
        navi_mode_ != NAVI_MODE::REFERENCE_PATH && rolling_route.size() >= 2;
    bool rolling_target_is_corner = false;
    double rolling_corner_angle = 0.0;
    Eigen::Vector3d rolling_corner_velocity_direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d rolling_target_direction = Eigen::Vector3d::Zero();
    double rolling_target_arc_length = 0.0;

    auto sampleRollingRoute = [&](double requested_arc_length,
                                  Eigen::Vector3d &point,
                                  Eigen::Vector3d &target_direction,
                                  bool &target_is_corner,
                                  double &corner_angle,
                                  Eigen::Vector3d &corner_velocity_direction) {
      if (!use_rolling_route || requested_arc_length < 0.0)
        return false;

      double remaining = requested_arc_length;
      Eigen::Vector3d last_valid_direction = Eigen::Vector3d::Zero();
      for (size_t i = 1; i < rolling_route.size(); ++i)
      {
        const Eigen::Vector3d segment = rolling_route[i] - rolling_route[i - 1];
        const double segment_length = segment.norm();
        if (segment_length < 1e-6)
          continue;

        last_valid_direction = segment / segment_length;
        last_valid_direction(2) = 0.0;
        if (last_valid_direction.norm() > 1e-6)
          last_valid_direction.normalize();

        if (remaining < segment_length - 1e-6)
        {
          point = rolling_route[i - 1] +
                  segment * (remaining / segment_length);
          target_direction = last_valid_direction;
          target_is_corner = false;
          corner_angle = 0.0;
          corner_velocity_direction.setZero();
          return true;
        }

        remaining -= segment_length;
        if (remaining <= 1e-6)
        {
          point = rolling_route[i];
          target_direction = last_valid_direction;
          target_is_corner = i + 1 < rolling_route.size();
          corner_angle = 0.0;
          corner_velocity_direction.setZero();

          if (target_is_corner)
          {
            Eigen::Vector3d outgoing = rolling_route[i + 1] - rolling_route[i];
            outgoing(2) = 0.0;
            if (target_direction.norm() > 1e-3 && outgoing.norm() > 1e-3)
            {
              outgoing.normalize();
              corner_angle = std::acos(std::clamp(
                  target_direction.dot(outgoing), -1.0, 1.0));
              corner_velocity_direction = target_direction + outgoing;
              if (corner_velocity_direction.norm() > 1e-3)
                corner_velocity_direction.normalize();
            }
            else
            {
              target_is_corner = false;
            }
          }
          return true;
        }
      }

      point = rolling_route.back();
      target_direction = last_valid_direction;
      target_is_corner = false;
      corner_angle = 0.0;
      corner_velocity_direction.setZero();
      return target_direction.norm() > 1e-3;
    };

    if (use_rolling_route)
    {
      // Look ahead by arc length on the collision-checked polyline instead of
      // ending every local B-spline at its first lattice corner.  This keeps
      // the full planning horizon while preserving the A-star detour shape.
      double route_length = 0.0;
      for (size_t i = 1; i < rolling_route.size(); ++i)
        route_length += (rolling_route[i] - rolling_route[i - 1]).norm();
      rolling_target_arc_length = std::min(planning_horizon_, route_length);
      if (rolling_target_arc_length < 1e-3 ||
          !sampleRollingRoute(
              rolling_target_arc_length, local_target_pt_,
              rolling_target_direction, rolling_target_is_corner,
              rolling_corner_angle, rolling_corner_velocity_direction))
        return false;
    }
    else
    {
      double t;
      for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
      {
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
        double dist = (pos_t - start_pt_).norm();

        if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                       dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
          local_target_pt_ = pos_t;
          target_t = t;
          planner_manager_->global_data_.last_progress_time_ = t;
          break;
        }
        if (dist < dist_min)
        {
          dist_min = dist;
          dist_min_t = t;
        }
        if (dist >= planning_horizon_)
        {
          local_target_pt_ = pos_t;
          target_t = t;
          planner_manager_->global_data_.last_progress_time_ = dist_min_t;
          break;
        }
      }
      if (t >= planner_manager_->global_data_.global_duration_) // Last global point
      {
        local_target_pt_ = end_pt_;
        target_t = planner_manager_->global_data_.global_duration_;
      }
    }

    auto targetRequiredClearance = [&](const Eigen::Vector3d &pt) {
      if (!use_rolling_route)
        return local_target_clearance_;

      const double yaw = rolling_target_direction.head<2>().norm() > 1e-3
                             ? std::atan2(
                                   rolling_target_direction(1),
                                   rolling_target_direction(0))
                             : estimateYawFromSegment(start_pt_, pt);
      auto map = planner_manager_->grid_map_;
      const bool start_is_nominally_free =
          map->getInflateOccupancy(start_pt_, yaw) == 0;
      const bool start_lacks_full_clearance =
          map->getInflateOccupancyWithClearance(
              start_pt_, yaw, local_target_clearance_) != 0;
      if (!start_is_nominally_free || !start_lacks_full_clearance)
        return local_target_clearance_;

      // Match the rolling-route and trajectory validators: when tracking has
      // left the robot inside the safety margin (but outside the obstacle), a
      // short target only has to improve clearance proportionally. Requiring
      // the full margin at a 10 cm A-star recovery point caused an otherwise
      // valid exit route to be rejected hundreds of times.
      const double recovery_ratio = std::min(
          1.0, (pt - start_pt_).norm() /
                   planner_manager_->pp_.clearance_recovery_distance_);
      return local_target_clearance_ * recovery_ratio;
    };
    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      const double target_yaw =
          use_rolling_route && rolling_target_direction.head<2>().norm() > 1e-3
              ? std::atan2(
                    rolling_target_direction(1), rolling_target_direction(0))
              : estimateYawFromSegment(start_pt_, pt);
      return planner_manager_->grid_map_->getInflateOccupancyWithClearance(
          pt, target_yaw, targetRequiredClearance(pt));
    };
    auto withinPlanningHorizon = [&](const Eigen::Vector3d &pt) {
      return (pt - start_pt_).norm() <= planning_horizon_ + 1e-3;
    };
    if (targetOccupancy(local_target_pt_) != 0)
    {
      if (use_rolling_route)
      {
        // Back off along the already verified A-star polyline.  Once the local
        // target spans more than one route segment, backing off on the direct
        // start-to-target chord could cut across the obstacle being avoided.
        if (rolling_target_arc_length < 0.25)
          return false;
        bool found_backoff = false;
        for (double backoff = 0.05;
             backoff <= rolling_target_arc_length - 0.2;
             backoff += 0.05)
        {
          Eigen::Vector3d candidate;
          Eigen::Vector3d candidate_direction;
          Eigen::Vector3d candidate_corner_direction;
          bool candidate_is_corner = false;
          double candidate_corner_angle = 0.0;
          const double candidate_arc_length =
              rolling_target_arc_length - backoff;
          if (!sampleRollingRoute(
                  candidate_arc_length, candidate, candidate_direction,
                  candidate_is_corner, candidate_corner_angle,
                  candidate_corner_direction))
            continue;

          rolling_target_direction = candidate_direction;
          if (targetOccupancy(candidate) == 0)
          {
            local_target_pt_ = candidate;
            rolling_target_arc_length = candidate_arc_length;
            rolling_target_is_corner = candidate_is_corner;
            rolling_corner_angle = candidate_corner_angle;
            rolling_corner_velocity_direction = candidate_corner_direction;
            found_backoff = true;
            break;
          }
        }
        if (!found_backoff)
          return false;
      }
      else
      {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (withinPlanningHorizon(pt) && targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (withinPlanningHorizon(pt) && targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
        return false;
      }
      }
    }

    if (targetOccupancy(local_target_pt_) != 0)
      return false;

    const double accepted_target_clearance =
        targetRequiredClearance(local_target_pt_);
    if (accepted_target_clearance + 1e-3 < local_target_clearance_)
    {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Using a short clearance-recovery target (%.2f m required now, %.2f m nominal)",
          accepted_target_clearance, local_target_clearance_);
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else if (use_rolling_route)
    {
      if (rolling_target_is_corner)
      {
        // Carry velocity through ordinary A-star corners, reducing it as the
        // turn grows.  Only a genuinely large corner asks the controller to
        // stop translating so it can realign the body first.
        if (rolling_corner_angle >= rolling_corner_stop_angle_ ||
            rolling_corner_velocity_direction.norm() < 1e-3)
        {
          local_target_vel_.setZero();
        }
        else
        {
          const double speed_ratio = std::cos(
              0.5 * M_PI * rolling_corner_angle /
              rolling_corner_stop_angle_);
          local_target_vel_ = rolling_corner_velocity_direction *
                              planner_manager_->pp_.max_vel_ *
                              std::max(0.0, speed_ratio);
          RCLCPP_INFO_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Carrying %.2f m/s through a %.0f deg rolling-route corner",
              local_target_vel_.norm(), rolling_corner_angle * 180.0 / M_PI);
        }
      }
      else
      {
        Eigen::Vector3d direction = rolling_target_direction;
        if (direction.norm() < 1e-3)
          direction = local_target_pt_ - start_pt_;
        if (direction.norm() > 1e-3)
          local_target_vel_ = direction.normalized() * planner_manager_->pp_.max_vel_;
        else
          local_target_vel_.setZero();
      }
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      const Eigen::Vector3d to_local_target = local_target_pt_ - start_pt_;
      if (local_target_vel_.dot(to_local_target) < 0.0)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Global reference velocity points backwards; using zero local terminal velocity");
        local_target_vel_.setZero();
      }
      else if (local_target_vel_.norm() > planner_manager_->pp_.max_vel_)
      {
        local_target_vel_ = local_target_vel_.normalized() * planner_manager_->pp_.max_vel_;
      }
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
