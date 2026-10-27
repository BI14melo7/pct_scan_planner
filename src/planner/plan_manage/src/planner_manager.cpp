// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <chrono>
#include <limits>
#include <thread>

namespace scan_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    pp_.execution_clearance_ = std::max(0.0, get_double("manager.execution_clearance", 0.15));
    pp_.clearance_recovery_distance_ = std::max(
        0.1, get_double("manager.clearance_recovery_distance", 0.6));
    global_astar_resolution_ = std::max(
        0.05, get_double("manager.global_astar_resolution", 0.1));
    global_route_clearance_ = std::max(
        pp_.execution_clearance_, get_double("manager.global_route_clearance", 0.25));

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    // Do not tie the A-star detour window to the local trajectory length.  A
    // short local plan is easier to optimize, while the search still needs a
    // wide enough window to find a route around long walls.
    const double astar_search_extent = std::max(
        5.0, get_double("manager.astar_search_extent", 19.0));
    const int astar_pool_voxels = std::max(
        100, static_cast<int>(std::ceil(astar_search_extent / grid_map_->getResolution())));
    bspline_optimizer_rebound_->a_star_->initGridMap(
        grid_map_, Eigen::Vector3i(astar_pool_voxels, astar_pool_voxels, astar_pool_voxels),
        global_route_clearance_);
    RCLCPP_INFO(node_->get_logger(),
                "A-star detour window: %.2f m (%d cells), route clearance: %.2f m",
                astar_pool_voxels * grid_map_->getResolution(), astar_pool_voxels,
                global_route_clearance_);

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    // Clearance recovery can intentionally select the first free A-star cell
    // only 10--20 cm away.  Treating every such segment as "already at goal"
    // leaves the robot parked inside the safety margin with no way to exit.
    if ((start_pt - local_target_pt).norm() < 0.08)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    // The upstream feasibility/refinement stage above is authoritative for
    // B-spline timing.  Keep the additional hard gate focused on spatial
    // safety; a second dynamics model here previously rejected valid refined
    // turns and canceled otherwise collision-free navigation tasks.
    if (!flag_step_2_success || !checkCollisionFree(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::planGlobalTrajWithLocalDetour(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pos,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    global_route_points_.clear();
    const Eigen::Vector3d to_goal = end_pos - start_pos;
    const double goal_distance = to_goal.norm();
    if (goal_distance < 1e-3)
      return planGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);

    // The occupancy grid is rolling, so only search the currently observable
    // part of the global route.  This route is rebuilt at every local replan;
    // newly observed obstacles therefore become global detour waypoints before
    // the local optimizer is asked to track them.
    const double lookahead = std::min(pp_.planning_horizon_, goal_distance);
    const Eigen::Vector3d local_goal = start_pos + to_goal * (lookahead / goal_distance);
    const Eigen::Vector3d line_delta = local_goal - start_pos;
    const double line_length = line_delta.norm();
    const double line_yaw = std::atan2(line_delta(1), line_delta(0));
    bool direct_path_blocked = false;
    const int line_samples = std::max(
        1, static_cast<int>(std::ceil(line_length / grid_map_->getResolution())));
    for (int i = 1; i <= line_samples; ++i)
    {
      const Eigen::Vector3d sample =
          start_pos + line_delta * (static_cast<double>(i) / line_samples);
      if (grid_map_->getInflateOccupancyWithClearance(
              sample, line_yaw, global_route_clearance_) != 0)
      {
        direct_path_blocked = true;
        break;
      }
    }

    if (!direct_path_blocked)
    {
      global_route_points_.push_back(start_pos);
      global_route_points_.push_back(end_pos);
      return planGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
    }

    if (!bspline_optimizer_rebound_ || !bspline_optimizer_rebound_->a_star_)
      return false;

    std::vector<Eigen::Vector3d> astar_path;
    auto tryAstarGoal = [&](const Eigen::Vector3d &candidate,
                            bool require_exact_free,
                            std::vector<Eigen::Vector3d> &path) {
      const Eigen::Vector3d delta = candidate - start_pos;
      if (delta.norm() < 0.2)
        return false;

      if (require_exact_free)
      {
        const double yaw = std::atan2(delta(1), delta(0));
        if (grid_map_->getInflateOccupancyWithClearance(
                candidate, yaw, global_route_clearance_) != 0)
          return false;
      }

      if (bspline_optimizer_rebound_->a_star_->AstarSearch(
              global_astar_resolution_, start_pos, candidate) != ASTAR_RET::SUCCESS)
        return false;

      path = bspline_optimizer_rebound_->a_star_->getPath();
      return path.size() >= 2;
    };

    bool used_lateral_proxy = false;
    double proxy_angle_degrees = 0.0;
    double proxy_radius = line_delta.head<2>().norm();
    if (!tryAstarGoal(local_goal, false, astar_path))
    {
      // A straight rolling endpoint can lie in front of an obstacle corner.
      // Moving it to the geometrically nearest free cell is not sufficient:
      // that cell can be a disconnected sliver between two inflated
      // obstacles. Search reachable goals on progressively wider lateral
      // arcs instead, while still making progress toward the real target.
      const Eigen::Vector2d desired_xy = line_delta.head<2>();
      const double desired_xy_length = desired_xy.norm();
      const double pi = std::acos(-1.0);
      const double radius_scales[] = {1.0, 0.8, 0.6};
      const double angle_degrees[] = {15.0, 30.0, 45.0, 60.0, 75.0};
      double best_score = std::numeric_limits<double>::infinity();

      if (desired_xy_length > 0.5)
      {
        const Eigen::Vector2d forward = desired_xy / desired_xy_length;
        for (double radius_scale : radius_scales)
        {
          bool found_at_radius = false;
          for (double angle_deg : angle_degrees)
          {
            bool found_at_angle = false;
            for (double sign : {-1.0, 1.0})
            {
              const double angle = sign * angle_deg * pi / 180.0;
              Eigen::Matrix2d rotation;
              rotation << std::cos(angle), -std::sin(angle),
                          std::sin(angle),  std::cos(angle);

              Eigen::Vector3d candidate = start_pos;
              const double radius = desired_xy_length * radius_scale;
              candidate.head<2>() += rotation * forward * radius;
              candidate(2) = start_pos(2) +
                             radius_scale * (local_goal(2) - start_pos(2));

              std::vector<Eigen::Vector3d> candidate_path;
              if (!tryAstarGoal(candidate, true, candidate_path))
                continue;

              double path_length = 0.0;
              for (size_t i = 1; i < candidate_path.size(); ++i)
                path_length += (candidate_path[i] - candidate_path[i - 1]).norm();
              const double score = path_length + (end_pos - candidate_path.back()).norm();
              if (score < best_score)
              {
                best_score = score;
                astar_path = std::move(candidate_path);
                proxy_angle_degrees = sign * angle_deg;
                proxy_radius = radius;
              }
              found_at_angle = true;
              found_at_radius = true;
            }

            // Prefer the smallest heading change with a reachable candidate;
            // compare both sides at that angle, then stop widening the arc.
            if (found_at_angle)
              break;
          }
          if (found_at_radius)
          {
            used_lateral_proxy = true;
            break;
          }
        }
      }
    }

    if (astar_path.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Rolling global A-star could not find a reachable forward or lateral goal in the %.2f m lookahead",
                  lookahead);
      return false;
    }

    if (used_lateral_proxy)
      RCLCPP_INFO(node_->get_logger(),
                  "Straight lookahead goal was unreachable; using a reachable lateral proxy (angle %.0f deg, radius %.2f m)",
                  proxy_angle_degrees, proxy_radius);

    std::vector<Eigen::Vector3d> raw_path;
    raw_path.reserve(astar_path.size() + 1);
    raw_path.push_back(start_pos);
    for (const auto &point : astar_path)
    {
      if ((point - raw_path.back()).norm() > 1e-3)
        raw_path.push_back(point);
    }

    // A-star may move an occupied lookahead endpoint to the nearest free
    // cell.  Keep that actual free endpoint; reconnecting the original point
    // would put the global route straight back inside the obstacle.
    const Eigen::Vector3d route_end = raw_path.back();

    std::vector<double> arc_length(raw_path.size(), 0.0);
    for (size_t i = 1; i < raw_path.size(); ++i)
      arc_length[i] = arc_length[i - 1] + (raw_path[i] - raw_path[i - 1]).norm();

    const double detour_length = arc_length.back();
    if (detour_length < 1e-3)
      return false;

    auto segmentIsFree = [&](const Eigen::Vector3d &from,
                             const Eigen::Vector3d &to,
                             bool allow_start_recovery) {
      const Eigen::Vector3d delta = to - from;
      const double length = delta.norm();
      if (length < 1e-6)
        return true;
      const double yaw = std::atan2(delta(1), delta(0));
      const int samples = std::max(
          1, static_cast<int>(std::ceil(length / grid_map_->getResolution())));
      for (int i = 1; i <= samples; ++i)
      {
        const double ratio = static_cast<double>(i) / samples;
        double clearance = global_route_clearance_;
        if (allow_start_recovery)
        {
          const double traveled = length * ratio;
          clearance *= std::min(
              1.0, traveled / pp_.clearance_recovery_distance_);
        }
        if (grid_map_->getInflateOccupancyWithClearance(
                from + ratio * delta, yaw, clearance) != 0)
          return false;
      }
      return true;
    };

    // Remove the dense 10 cm A-star staircase.  Feeding every lattice point
    // into a minimum-snap fit creates oscillatory, even backwards, reference
    // velocities.  Retain only the farthest line-of-sight turn points.
    std::vector<Eigen::Vector3d> simplified_path;
    simplified_path.push_back(raw_path.front());
    size_t anchor = 0;
    while (anchor + 1 < raw_path.size())
    {
      size_t next = anchor + 1;
      for (size_t candidate = raw_path.size() - 1; candidate > anchor; --candidate)
      {
        if (segmentIsFree(raw_path[anchor], raw_path[candidate], anchor == 0))
        {
          next = candidate;
          break;
        }
      }
      simplified_path.push_back(raw_path[next]);
      anchor = next;
    }

    global_route_points_ = simplified_path;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(simplified_path.size());
    for (size_t i = 1; i < simplified_path.size(); ++i)
      waypoints.push_back(simplified_path[i]);

    if ((end_pos - route_end).norm() > 1e-3)
      waypoints.push_back(end_pos);

    RCLCPP_INFO(node_->get_logger(),
                "Rolling global route: direct %.2f m is blocked, A-star detour %.2f m simplified to %zu segments (%zu interior corners)",
                line_length, detour_length, simplified_path.size() - 1,
                simplified_path.size() > 2 ? simplified_path.size() - 2 : 0);
    return planGlobalTrajWaypoints(
        start_pos, start_vel, start_acc, waypoints, end_vel, end_acc);
  }

  bool SCANPlannerManager::checkCollisionFree(UniformBspline position_traj)
  {
    const double duration = position_traj.getTimeSum();
    if (duration <= 0.0)
      return false;

    // At most half a voxel of nominal travel per sample.  This validates the
    // complete trajectory that will be published, including its terminal
    // third (which the upstream receding-horizon implementation skipped).
    const double speed = std::max(pp_.max_vel_, 0.1);
    const double sample_dt = std::max(
        0.005, std::min(0.05, grid_map_->getResolution() / (2.0 * speed)));

    const Eigen::Vector3d start = position_traj.evaluateDeBoorT(0.0);
    const Eigen::Vector3d start_next = position_traj.evaluateDeBoorT(
        std::min(sample_dt, duration));
    const Eigen::Vector2d start_delta = start_next.head<2>() - start.head<2>();
    const double start_yaw = start_delta.squaredNorm() > 1e-8
                                 ? std::atan2(start_delta(1), start_delta(0))
                                 : 0.0;
    if (grid_map_->getInflateOccupancy(start, start_yaw) != 0)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Reject trajectory: initial robot footprint is in collision");
      return false;
    }
    const bool recovering_clearance =
        grid_map_->getInflateOccupancyWithClearance(
            start, start_yaw, pp_.execution_clearance_) != 0;

    for (double t = 0.0; t <= duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      const double prev_t = std::max(tc - sample_dt, 0.0);
      const double next_t = std::min(tc + sample_dt, duration);
      const Eigen::Vector3d pos = position_traj.evaluateDeBoorT(tc);
      const Eigen::Vector3d prev = position_traj.evaluateDeBoorT(prev_t);
      const Eigen::Vector3d next = position_traj.evaluateDeBoorT(next_t);
      const Eigen::Vector2d delta = next.head<2>() - prev.head<2>();
      const double yaw = delta.squaredNorm() > 1e-8
                             ? std::atan2(delta(1), delta(0))
                             : 0.0;
      double required_clearance = pp_.execution_clearance_;
      if (recovering_clearance && tc < duration - 1e-6)
      {
        const double displacement = (pos.head<2>() - start.head<2>()).norm();
        const double recovery_ratio = std::min(
            1.0, displacement / pp_.clearance_recovery_distance_);
        required_clearance *= recovery_ratio;
      }
      const int occupancy = grid_map_->getInflateOccupancyWithClearance(
          pos, yaw, required_clearance);
      if (occupancy != 0)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Reject trajectory: sample violates %.2f m required clearance at "
                    "t=%.3f, p=[%.2f, %.2f, %.2f], occupancy=%d",
                    required_clearance, tc, pos(0), pos(1), pos(2), occupancy);
        return false;
      }
    }
    if (recovering_clearance)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "Accepted clearance-recovery trajectory; full %.2f m margin "
                  "must be regained within %.2f m",
                  pp_.execution_clearance_, pp_.clearance_recovery_distance_);
    }
    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
