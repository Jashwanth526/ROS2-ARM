#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <vector>
#include <mutex>
#include <deque>
#include <array>
#include <unordered_map>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

namespace arduinobot_remote
{
  class TaskServerSimple : public rclcpp::Node
  {
  public:
    explicit TaskServerSimple(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("task_server_simple", options)
    {
      RCLCPP_INFO(get_logger(), "Starting Simple Task Server");

      this->declare_parameter<std::string>("target_color", "any");
      this->declare_parameter<bool>("post_confirm_updates", false);
      this->declare_parameter<double>("scan_min", -1.2);
      this->declare_parameter<double>("scan_max", 1.2);
      this->declare_parameter<int>("scan_steps", 13);
      this->declare_parameter<int>("scan_dwell_cycles", 10);
      this->declare_parameter<double>("scan_dwell_interval", 0.4);
      this->declare_parameter<double>("grasp_z_offset", 0.005);
      this->declare_parameter<double>("pregrasp_z_offset", 0.12);
      target_color_ = this->get_parameter("target_color").as_string();
      post_confirm_updates_ = this->get_parameter("post_confirm_updates").as_bool();
      scan_min_ = this->get_parameter("scan_min").as_double();
      scan_max_ = this->get_parameter("scan_max").as_double();
      scan_steps_ = this->get_parameter("scan_steps").as_int();
      scan_dwell_cycles_ = this->get_parameter("scan_dwell_cycles").as_int();
      scan_dwell_interval_ = this->get_parameter("scan_dwell_interval").as_double();
      grasp_z_offset_ = this->get_parameter("grasp_z_offset").as_double();
      pregrasp_z_offset_ = this->get_parameter("pregrasp_z_offset").as_double();

      action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
          this, "task_server",
          std::bind(&TaskServerSimple::goalCallback, this, _1, _2),
          std::bind(&TaskServerSimple::cancelCallback, this, _1),
          std::bind(&TaskServerSimple::acceptedCallback, this, _1));

      auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      auto sub_options = rclcpp::SubscriptionOptions();
      sub_options.callback_group = callback_group;

      point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
          "detected_objects", 10,
          std::bind(&TaskServerSimple::objectCallback, this, _1), sub_options);

      detection_info_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "detection_info", 10,
          std::bind(&TaskServerSimple::detectionInfoCallback, this, _1), sub_options);

      RCLCPP_INFO(get_logger(), "Simple Task Server initialized");
    }

  private:
    rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_info_subscriber_;

    std::mutex object_mutex_;
    double object_x_ = 0.0, object_y_ = 0.0, object_z_ = 0.0;
    double confirmed_x_ = 0.0, confirmed_y_ = 0.0, confirmed_z_ = 0.0;
    std::atomic<bool> object_detected_{false};
    std::atomic<bool> pick_in_progress_{false};
    std::atomic<bool> continuous_grip_{false};
    std::string target_color_ = "any";
    std::string detected_color_ = "";
    bool post_confirm_updates_ = false;
    double scan_min_ = -1.2;
    double scan_max_ = 1.2;
    int scan_steps_ = 13;
    int scan_dwell_cycles_ = 10;
    double scan_dwell_interval_ = 0.4;
    double grasp_z_offset_ = 0.005;
    double pregrasp_z_offset_ = 0.12;

    std::deque<std::array<double, 3>> target_samples_;
    const size_t max_samples_ = 10;
    const size_t min_samples_confirm_ = 6;
    const double var_thresh_xy_ = 0.02;
    const double var_thresh_z_ = 0.03;

    void initializeMoveIt()
    {
      if (!arm_move_group_)
      {
        RCLCPP_INFO(get_logger(), "Initializing MoveIt interfaces...");
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
        object_detected_.store(false);
        target_samples_.clear();
        arm_move_group_->setMaxVelocityScalingFactor(0.3);
        arm_move_group_->setMaxAccelerationScalingFactor(0.3);
        arm_move_group_->setGoalPositionTolerance(0.01);
        arm_move_group_->setGoalOrientationTolerance(0.1);
        arm_move_group_->setPlanningTime(15.0);
        arm_move_group_->setNumPlanningAttempts(3);

        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        if (!tf_buffer_)
        {
          tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
          tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        }

        RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized with approximate IK enabled");
      }
    }

    bool computeTcpOffsetBase(Eigen::Vector3d &offset_base)
    {
      (void)offset_base;
      return false;
    }

    void objectCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      const std::string &fid = msg->header.frame_id;
      bool is_base = (fid == "base_link") || (fid.rfind("base_link/", 0) == 0);
      if (!is_base)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received point in frame '%s' (expected base_link or base_link/<color>)", fid.c_str());
      }

      std::string coord_color;
      if (fid.rfind("base_link/", 0) == 0 && fid.size() > std::string("base_link/").size())
      {
        coord_color = fid.substr(std::string("base_link/").size());
      }

      bool color_ok = false;
      if (target_color_ == "any")
      {
        color_ok = true;
      }
      else if (!coord_color.empty())
      {
        color_ok = (coord_color == target_color_);
      }
      else
      {
        color_ok = (!detected_color_.empty() && detected_color_ == target_color_);
      }
      if (color_ok)
      {
        RCLCPP_INFO(get_logger(), "Object in %s: (%.3f, %.3f, %.3f)", fid.c_str(), msg->point.x, msg->point.y, msg->point.z);

        if (!object_detected_.load())
        {
          std::array<double, 3> p = {msg->point.x, msg->point.y, msg->point.z};
          if (target_samples_.size() >= max_samples_)
            target_samples_.pop_front();
          target_samples_.push_back(p);

          auto n = static_cast<double>(target_samples_.size());
          double sumx = 0, sumy = 0, sumz = 0, sumx2 = 0, sumy2 = 0, sumz2 = 0;
          for (const auto &q : target_samples_)
          {
            sumx += q[0];
            sumy += q[1];
            sumz += q[2];
            sumx2 += q[0] * q[0];
            sumy2 += q[1] * q[1];
            sumz2 += q[2] * q[2];
          }
          double meanx = sumx / n, meany = sumy / n, meanz = sumz / n;
          double varx = std::max(0.0, sumx2 / n - meanx * meanx);
          double vary = std::max(0.0, sumy2 / n - meany * meany);
          double varz = std::max(0.0, sumz2 / n - meanz * meanz);

          RCLCPP_INFO(get_logger(), "Sample buf=%zu mean(%.3f,%.3f,%.3f) var(%.4f,%.4f,%.4f)",
                      target_samples_.size(), meanx, meany, meanz, varx, vary, varz);

          if (target_samples_.size() >= min_samples_confirm_ && varx <= var_thresh_xy_ && vary <= var_thresh_xy_ && varz <= var_thresh_z_)
          {
            {
              std::lock_guard<std::mutex> lock(object_mutex_);
              object_x_ = meanx;
              object_y_ = meany;
              object_z_ = meanz;
              confirmed_x_ = meanx;
              confirmed_y_ = meany;
              confirmed_z_ = meanz;
            }
            object_detected_.store(true);
            if (detected_color_.empty() && !coord_color.empty())
            {
              detected_color_ = coord_color;
            }
            RCLCPP_INFO(get_logger(), "[DIAG] Target %s object CONFIRMED at (%.3f, %.3f, %.3f) from %zu samples (var %.4f, %.4f, %.4f)",
                        detected_color_.c_str(), meanx, meany, meanz, target_samples_.size(), varx, vary, varz);

            target_samples_.clear();
          }
        }
        else if (object_detected_.load())
        {
          if (!post_confirm_updates_)
          {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 4000,
                                 "[DIAG] Post-confirm updates disabled - ignoring detection (anchor (%.3f,%.3f,%.3f))",
                                 confirmed_x_, confirmed_y_, confirmed_z_);
            return;
          }
          double new_x = msg->point.x;
          double new_y = msg->point.y;
          double new_z = msg->point.z;

          double dx, dy, dz;
          {
            std::lock_guard<std::mutex> lock(object_mutex_);
            dx = new_x - confirmed_x_;
            dy = new_y - confirmed_y_;
            dz = new_z - confirmed_z_;
          }
          double dist_3d = std::sqrt(dx * dx + dy * dy + dz * dz);
          const double MAX_UPDATE_DIST = 0.03;

          if (dist_3d <= MAX_UPDATE_DIST)
          {
            std::lock_guard<std::mutex> lock(object_mutex_);
            object_x_ = new_x;
            object_y_ = new_y;
            object_z_ = new_z;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                 "[DIAG] Object pose updated to (%.3f, %.3f, %.3f) [dist from anchor: %.3fm]",
                                 new_x, new_y, new_z, dist_3d);
          }
          else
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[DIAG] REJECTED outlier at (%.3f,%.3f,%.3f) - dist %.3fm from CONFIRMED (%.3f,%.3f,%.3f)",
                                 new_x, new_y, new_z, dist_3d, confirmed_x_, confirmed_y_, confirmed_z_);
          }
        }
      }
    }

    void detectionInfoCallback(const std_msgs::msg::String::SharedPtr msg)
    {
      const std::string &data = msg->data;
      std::string current_color;
      if (data.find("red") != std::string::npos)
        current_color = "red";
      else if (data.find("blue") != std::string::npos)
        current_color = "blue";
      else if (data.find("green") != std::string::npos)
        current_color = "green";

      if (!current_color.empty() && (target_color_ == "any" || current_color == target_color_))
      {
        detected_color_ = current_color;
        RCLCPP_INFO(get_logger(), "Target %s detection hit", current_color.c_str());
      }
    }

    bool moveToJointPosition(const std::vector<double> &joint_goals)
    {
      (void)waitForStateStable(1.5, 0.002, 4);
      try
      {
        arm_move_group_->setStartStateToCurrentState();
      }
      catch (...)
      {
      }

      arm_move_group_->setJointValueTarget(joint_goals);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      int attempts = 0;
      while (attempts < 3)
      {
        auto code = arm_move_group_->plan(plan);
        if (code == moveit::core::MoveItErrorCode::SUCCESS)
        {
          if (!startStateDeviationOkay(plan, 0.01))
          {
            RCLCPP_WARN(get_logger(), "Start state deviation high after joint plan (attempt %d) - replanning", attempts + 1);
            try
            {
              arm_move_group_->setStartStateToCurrentState();
            }
            catch (...)
            {
            }
            attempts++;
            continue;
          }
          (void)waitForStateStable(0.7, 0.002, 3);
          if (!startStateDeviationOkay(plan, 0.01))
          {
            RCLCPP_WARN(get_logger(), "Post-settle deviation still high (joint plan) - replanning before execute");
            try
            {
              arm_move_group_->setStartStateToCurrentState();
            }
            catch (...)
            {
            }
            attempts++;
            continue;
          }
          arm_move_group_->execute(plan);
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          return true;
        }
        else
        {
          RCLCPP_WARN(get_logger(), "Joint planning failed (attempt %d) - retrying", attempts + 1);
          try
          {
            arm_move_group_->setStartStateToCurrentState();
          }
          catch (...)
          {
          }
          attempts++;
          continue;
        }
      }
      return false;
    }

    bool moveGripper(const std::vector<double> &gripper_goals)
    {
      (void)waitForStateStable(1.0, 0.002, 3);
      try
      {
        gripper_move_group_->setStartStateToCurrentState();
      }
      catch (...)
      {
      }

      gripper_move_group_->setJointValueTarget(gripper_goals);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      auto code = gripper_move_group_->plan(plan);
      if (code == moveit::core::MoveItErrorCode::SUCCESS)
      {
        (void)waitForStateStable(0.5, 0.002, 2);
        gripper_move_group_->execute(plan);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        return true;
      }
      return false;
    }

    void continuousGripperSqueeze()
    {
      const std::vector<double> max_close = {0.80, -0.80};
      RCLCPP_INFO(get_logger(), "[GRIP] Starting continuous squeeze thread");

      while (continuous_grip_.load() && rclcpp::ok())
      {
        try
        {
          gripper_move_group_->setJointValueTarget(max_close);
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          auto code = gripper_move_group_->plan(plan);
          if (code == moveit::core::MoveItErrorCode::SUCCESS)
          {
            gripper_move_group_->execute(plan);
          }
        }
        catch (...)
        {
          RCLCPP_WARN(get_logger(), "[GRIP] Exception in continuous squeeze - continuing");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(800));
      }

      RCLCPP_INFO(get_logger(), "[GRIP] Stopped continuous squeeze thread");
    }

    bool moveToPose(const geometry_msgs::msg::PoseStamped &target_pose, const std::string &ee_link)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      (void)waitForStateStable(1.5, 0.002, 4);
      try
      {
        arm_move_group_->setStartStateToCurrentState();
      }
      catch (...)
      {
      }

      arm_move_group_->setPositionTarget(
          target_pose.pose.position.x,
          target_pose.pose.position.y,
          target_pose.pose.position.z,
          ee_link);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      int attempts = 0;
      while (attempts < 3)
      {
        auto ok = arm_move_group_->plan(plan);
        if (ok == moveit::core::MoveItErrorCode::SUCCESS)
        {
          if (!startStateDeviationOkay(plan, 0.01))
          {
            RCLCPP_WARN(get_logger(), "Start state deviation high after pose plan (attempt %d) - replanning", attempts + 1);
            try
            {
              arm_move_group_->setStartStateToCurrentState();
            }
            catch (...)
            {
            }
            attempts++;
            continue;
          }
          (void)waitForStateStable(0.7, 0.002, 3);
          if (!startStateDeviationOkay(plan, 0.01))
          {
            RCLCPP_WARN(get_logger(), "Post-settle deviation still high (pose plan) - replanning before execute");
            try
            {
              arm_move_group_->setStartStateToCurrentState();
            }
            catch (...)
            {
            }
            attempts++;
            continue;
          }
          arm_move_group_->execute(plan);
          arm_move_group_->clearPoseTargets();
          rclcpp::sleep_for(std::chrono::milliseconds(500));
          return true;
        }
        else
        {
          RCLCPP_WARN(get_logger(), "Pose planning failed (attempt %d) - retrying", attempts + 1);
          try
          {
            arm_move_group_->setStartStateToCurrentState();
          }
          catch (...)
          {
          }
          attempts++;
          continue;
        }
      }
      arm_move_group_->clearPoseTargets();
      return false;
    }

    bool cartesianMoveToZ(double target_z, const std::string &ee_link)
    {
      if (!arm_move_group_)
        return false;
      try
      {
        arm_move_group_->setStartStateToCurrentState();
      }
      catch (...)
      {
      }
      arm_move_group_->setEndEffectorLink(ee_link);

      geometry_msgs::msg::PoseStamped current = arm_move_group_->getCurrentPose(ee_link);
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose intermediate = current.pose;
      geometry_msgs::msg::Pose goal = current.pose;

      goal.position.z = target_z;
      waypoints.push_back(intermediate);
      waypoints.push_back(goal);

      moveit_msgs::msg::RobotTrajectory traj;
      const double eef_step = 0.005;
      double fraction = arm_move_group_->computeCartesianPath(waypoints, eef_step, traj, true);
      RCLCPP_INFO(get_logger(), "[DIAG] Cartesian Z move fraction=%.3f (target_z=%.3f)", fraction, target_z);
      if (fraction < 0.95)
      {
        RCLCPP_WARN(get_logger(), "Cartesian Z path incomplete (%.2f%%) - aborting fallback", fraction * 100.0);
        return false;
      }
      auto code = arm_move_group_->execute(traj);
      bool ok = (code == moveit::core::MoveItErrorCode::SUCCESS);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
      return ok;
    }

    bool cartesianMoveToXY(double target_x, double target_y, const std::string &ee_link)
    {
      if (!arm_move_group_)
        return false;
      try
      {
        arm_move_group_->setStartStateToCurrentState();
      }
      catch (...)
      {
      }
      arm_move_group_->setEndEffectorLink(ee_link);

      geometry_msgs::msg::PoseStamped current = arm_move_group_->getCurrentPose(ee_link);
      std::vector<geometry_msgs::msg::Pose> waypoints;
      geometry_msgs::msg::Pose start = current.pose;
      geometry_msgs::msg::Pose goal = current.pose;
      goal.position.x = target_x;
      goal.position.y = target_y;
      waypoints.push_back(start);
      waypoints.push_back(goal);

      moveit_msgs::msg::RobotTrajectory traj;
      const double eef_step = 0.005;
      double fraction = arm_move_group_->computeCartesianPath(waypoints, eef_step, traj, true);
      RCLCPP_INFO(get_logger(), "[DIAG] Cartesian XY move fraction=%.3f (target=(%.3f,%.3f))", fraction, target_x, target_y);
      if (fraction < 0.95)
      {
        RCLCPP_WARN(get_logger(), "Cartesian XY path incomplete (%.2f%%) - aborting micro-centering", fraction * 100.0);
        return false;
      }
      auto code = arm_move_group_->execute(traj);
      bool ok = (code == moveit::core::MoveItErrorCode::SUCCESS);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      return ok;
    }

    bool waitForStateStable(double timeout_sec = 1.0, double max_delta = 0.002, int consecutive = 3)
    {
      if (!arm_move_group_)
        return false;
      auto start = std::chrono::steady_clock::now();
      std::vector<double> prev = arm_move_group_->getCurrentJointValues();
      int ok_count = 0;
      while (true)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::vector<double> curr = arm_move_group_->getCurrentJointValues();
        double maxdiff = 0.0;
        size_t n = std::min(prev.size(), curr.size());
        for (size_t i = 0; i < n; ++i)
        {
          double d = std::fabs(curr[i] - prev[i]);
          if (d > maxdiff)
            maxdiff = d;
        }
        if (maxdiff <= max_delta)
        {
          ok_count++;
          if (ok_count >= consecutive)
            return true;
        }
        else
        {
          ok_count = 0;
        }
        prev = curr;
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_sec)
          return false;
      }
    }

    bool startStateDeviationOkay(const moveit::planning_interface::MoveGroupInterface::Plan &plan, double tolerance)
    {
      const auto &traj = plan.trajectory.joint_trajectory;
      if (traj.points.empty() || traj.joint_names.empty())
        return true;

      const std::vector<std::string> group_names = arm_move_group_->getJointNames();
      const std::vector<double> current = arm_move_group_->getCurrentJointValues();
      if (group_names.size() != current.size())
        return true;

      std::unordered_map<std::string, size_t> name_to_index;
      name_to_index.reserve(traj.joint_names.size());
      for (size_t i = 0; i < traj.joint_names.size(); ++i)
        name_to_index[traj.joint_names[i]] = i;

      const auto &first = traj.points.front();
      double max_dev = 0.0;
      for (size_t i = 0; i < group_names.size(); ++i)
      {
        auto it = name_to_index.find(group_names[i]);
        if (it == name_to_index.end())
          continue;
        size_t idx = it->second;
        if (idx >= first.positions.size())
          continue;
        double dev = std::fabs(first.positions[idx] - current[i]);
        if (dev > max_dev)
          max_dev = dev;
      }
      RCLCPP_INFO(get_logger(), "[DIAG] Start state deviation check: max_dev=%.5f (tol=%.5f)", max_dev, tolerance);
      return max_dev <= tolerance;
    }

    std::string getEndEffectorLinkSafe()
    {
      std::string ee = arm_move_group_->getEndEffectorLink();
      if (ee.empty())
      {
        auto links = arm_move_group_->getLinkNames();
        if (!links.empty())
          ee = links.back();
      }
      return ee;
    }

    bool performScan()
    {
      RCLCPP_INFO(get_logger(), "Starting scan for %s objects...", target_color_.c_str());

      object_detected_.store(false);
      target_samples_.clear();
      pick_in_progress_.store(false);

      moveGripper({-0.7, 0.7});

      std::vector<std::vector<double>> scan_positions;
      int steps = std::max(3, scan_steps_);
      double base_min = std::min(scan_min_, scan_max_);
      double base_max = std::max(scan_min_, scan_max_);
      for (int i = 0; i < steps; ++i)
      {
        double t = (steps == 1) ? 0.5 : static_cast<double>(i) / (steps - 1);
        double base_angle = base_min + t * (base_max - base_min);
        scan_positions.push_back({base_angle, 0.3, -0.7});
      }

      RCLCPP_INFO(get_logger(), "Sweeping camera across workspace to find %s objects",
                  target_color_.c_str());

      for (size_t i = 0; i < scan_positions.size(); i++)
      {
        RCLCPP_INFO(get_logger(), "Scan position %zu/%zu: base=%.2f rad",
                    i + 1, scan_positions.size(), scan_positions[i][0]);
        moveToJointPosition(scan_positions[i]);

        const int dwell_checks = std::max(1, scan_dwell_cycles_);
        const int dwell_ms = static_cast<int>(std::round(std::max(0.05, scan_dwell_interval_) * 1000.0));
        for (int c = 0; c < dwell_checks; ++c)
        {
          if (object_detected_.load())
          {
            RCLCPP_INFO(get_logger(), "Confirmed target at position %zu (dwell %d/%d)", i + 1, c + 1, dwell_checks);
            break;
          }
          rclcpp::sleep_for(std::chrono::milliseconds(dwell_ms));
        }
        if (object_detected_.load())
        {
          RCLCPP_INFO(get_logger(), "Target object found! Stopping scan at position %zu/%zu", i + 1, scan_positions.size());
          break;
        }
      }

      if (object_detected_.load())
      {
        RCLCPP_INFO(get_logger(), "Scan complete - %s object detected", target_color_.c_str());
        return true;
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Scan complete - no %s object found", target_color_.c_str());
        return false;
      }
    }

    void performPick()
    {
      RCLCPP_INFO(get_logger(), "Performing pick sequence...");

      pick_in_progress_.store(true);

      if (!object_detected_.load())
      {
        RCLCPP_ERROR(get_logger(), "No object detected - cannot pick!");
        pick_in_progress_.store(false);
        return;
      }

      double obj_x, obj_y, obj_z;
      {
        std::lock_guard<std::mutex> lock(object_mutex_);
        obj_x = object_x_;
        obj_y = object_y_;
        obj_z = object_z_;
      }

      RCLCPP_INFO(get_logger(), "[DIAG] Locked object at pick start: (%.3f, %.3f, %.3f)",
                  obj_x, obj_y, obj_z);

      double horizontal_distance = std::sqrt(obj_x * obj_x + obj_y * obj_y);
      double base_angle = std::atan2(obj_y, obj_x);

      RCLCPP_INFO(get_logger(), "Object distance: %.3fm, angle: %.3f rad (%.1f deg)",
                  horizontal_distance, base_angle, base_angle * 180.0 / M_PI);

      if (horizontal_distance < 0.25 || horizontal_distance > 2.0)
      {
        RCLCPP_WARN(get_logger(), "[REACH] Object at %.3fm appears outside nominal range (0.25-2.0m); attempting plan anyway",
                    horizontal_distance);
      }

      RCLCPP_INFO(get_logger(), "Opening gripper");
      moveGripper({-0.7, 0.7});

      const std::string planning_frame = arm_move_group_->getPlanningFrame();
      const std::string ee_link = "tcp_link";

      geometry_msgs::msg::PoseStamped pregrasp;
      pregrasp.header.frame_id = planning_frame;
      pregrasp.pose.position.x = obj_x;
      pregrasp.pose.position.y = obj_y;
      pregrasp.pose.position.z = obj_z + pregrasp_z_offset_;

      geometry_msgs::msg::PoseStamped grasp;
      grasp.header.frame_id = planning_frame;
      grasp.pose.position.x = obj_x;
      grasp.pose.position.y = obj_y;
      grasp.pose.position.z = obj_z + grasp_z_offset_;

      RCLCPP_INFO(get_logger(), "[DIAG] Planning to pre-grasp pose (x=%.3f,y=%.3f,z=%.3f)",
                  pregrasp.pose.position.x, pregrasp.pose.position.y, pregrasp.pose.position.z);
      bool pose_ok = moveToPose(pregrasp, ee_link);
      if (!pose_ok)
      {
        RCLCPP_WARN(get_logger(), "Pre-grasp pose planning failed - falling back to joint heuristic");
        std::vector<double> current_joints = arm_move_group_->getCurrentJointValues();
        std::vector<double> point_joints = {base_angle, current_joints[1], current_joints[2]};
        moveToJointPosition(point_joints);
      }

      RCLCPP_INFO(get_logger(), "[DIAG] Planning to grasp pose (x=%.3f,y=%.3f,z=%.3f)",
                  grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z);
      RCLCPP_INFO(get_logger(), "[DIAG] Object coords at grasp plan (frozen): (%.3f, %.3f, %.3f)", obj_x, obj_y, obj_z);
      grasp.pose.position.x = obj_x;
      grasp.pose.position.y = obj_y;
      grasp.pose.position.z = obj_z + grasp_z_offset_;
      if (!moveToPose(grasp, ee_link))
      {
        RCLCPP_WARN(get_logger(), "Grasp pose planning failed - trying Cartesian vertical descent");
        (void)cartesianMoveToZ(grasp.pose.position.z, ee_link);
        if (!waitForStateStable(0.5, 0.003, 2))
        {
          auto joints = arm_move_group_->getCurrentJointValues();
          if (joints.size() >= 3)
          {
            joints[1] -= 0.05;
            joints[2] -= 0.05;
            (void)moveToJointPosition(joints);
          }
        }
      }

      if (tf_buffer_)
      {
        try
        {
          geometry_msgs::msg::TransformStamped T_b_L = tf_buffer_->lookupTransform(
              "base_link", "gripper_left", tf2::TimePointZero, std::chrono::milliseconds(200));
          geometry_msgs::msg::TransformStamped T_b_R = tf_buffer_->lookupTransform(
              "base_link", "gripper_right", tf2::TimePointZero, std::chrono::milliseconds(200));
          Eigen::Vector3d p_b_L(T_b_L.transform.translation.x,
                                T_b_L.transform.translation.y,
                                T_b_L.transform.translation.z);
          Eigen::Vector3d p_b_R(T_b_R.transform.translation.x,
                                T_b_R.transform.translation.y,
                                T_b_R.transform.translation.z);
          Eigen::Vector3d p_center = 0.5 * (p_b_L + p_b_R);
          RCLCPP_INFO(get_logger(), "[DIAG] Pre-grasp center: (%.3f, %.3f, %.3f) vs object (%.3f, %.3f, %.3f)",
                      p_center.x(), p_center.y(), p_center.z(), obj_x, obj_y, obj_z);

          double dx = obj_x - p_center.x();
          double dy = obj_y - p_center.y();
          double err_xy = std::sqrt(dx * dx + dy * dy);
          const double xy_tol = 0.015;
          if (err_xy > xy_tol)
          {
            RCLCPP_INFO(get_logger(), "[DIAG] XY error %.3fm > tol %.3fm -> micro-centering", err_xy, xy_tol);
            geometry_msgs::msg::TransformStamped T_b_TCP = tf_buffer_->lookupTransform(
                "base_link", ee_link, tf2::TimePointZero, std::chrono::milliseconds(200));
            Eigen::Vector3d p_b_TCP(T_b_TCP.transform.translation.x,
                                    T_b_TCP.transform.translation.y,
                                    T_b_TCP.transform.translation.z);
            Eigen::Vector3d d_tcp_center = p_b_TCP - p_center;
            double target_tcp_x = obj_x + d_tcp_center.x();
            double target_tcp_y = obj_y + d_tcp_center.y();
            (void)cartesianMoveToXY(target_tcp_x, target_tcp_y, ee_link);
            geometry_msgs::msg::TransformStamped T2_b_L = tf_buffer_->lookupTransform(
                "base_link", "gripper_left", tf2::TimePointZero, std::chrono::milliseconds(200));
            geometry_msgs::msg::TransformStamped T2_b_R = tf_buffer_->lookupTransform(
                "base_link", "gripper_right", tf2::TimePointZero, std::chrono::milliseconds(200));
            Eigen::Vector3d p2_b_L(T2_b_L.transform.translation.x,
                                   T2_b_L.transform.translation.y,
                                   T2_b_L.transform.translation.z);
            Eigen::Vector3d p2_b_R(T2_b_R.transform.translation.x,
                                   T2_b_R.transform.translation.y,
                                   T2_b_R.transform.translation.z);
            Eigen::Vector3d p2_center = 0.5 * (p2_b_L + p2_b_R);
            double d2 = std::hypot(obj_x - p2_center.x(), obj_y - p2_center.y());
            RCLCPP_INFO(get_logger(), "[DIAG] Post-centering center: (%.3f, %.3f, %.3f), err_xy=%.3fm",
                        p2_center.x(), p2_center.y(), p2_center.z(), d2);
            p_center = p2_center;
          }

          double desired_mid_z = obj_z + grasp_z_offset_;
          double mid_z_err = p_center.z() - desired_mid_z;
          const double z_tol = 0.01;
          if (std::fabs(mid_z_err) > z_tol)
          {
            geometry_msgs::msg::TransformStamped T_b_TCP2 = tf_buffer_->lookupTransform(
                "base_link", ee_link, tf2::TimePointZero, std::chrono::milliseconds(200));
            Eigen::Vector3d p_b_TCP2(T_b_TCP2.transform.translation.x,
                                     T_b_TCP2.transform.translation.y,
                                     T_b_TCP2.transform.translation.z);
            Eigen::Vector3d current_offset = p_b_TCP2 - p_center;
            double target_tcp_z = desired_mid_z + current_offset.z();
            RCLCPP_INFO(get_logger(), "[DIAG] Z error %.3fm -> micro-centering Z (target_tcp_z=%.3f)", mid_z_err, target_tcp_z);
            (void)cartesianMoveToZ(target_tcp_z, ee_link);
            geometry_msgs::msg::TransformStamped T3_b_L = tf_buffer_->lookupTransform(
                "base_link", "gripper_left", tf2::TimePointZero, std::chrono::milliseconds(200));
            geometry_msgs::msg::TransformStamped T3_b_R = tf_buffer_->lookupTransform(
                "base_link", "gripper_right", tf2::TimePointZero, std::chrono::milliseconds(200));
            Eigen::Vector3d p3_b_L(T3_b_L.transform.translation.x,
                                   T3_b_L.transform.translation.y,
                                   T3_b_L.transform.translation.z);
            Eigen::Vector3d p3_b_R(T3_b_R.transform.translation.x,
                                   T3_b_R.transform.translation.y,
                                   T3_b_R.transform.translation.z);
            Eigen::Vector3d p3_center = 0.5 * (p3_b_L + p3_b_R);
            double mid_z_err2 = p3_center.z() - desired_mid_z;
            RCLCPP_INFO(get_logger(), "[DIAG] Post-centering Z midpoint: (%.3f, %.3f, %.3f) mid_z_err=%.3fm",
                        p3_center.x(), p3_center.y(), p3_center.z(), mid_z_err2);
          }
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN(get_logger(), "TF diag failed before grasp: %s", ex.what());
        }
      }

      RCLCPP_INFO(get_logger(), "Grasping object with maximum force");
      moveGripper({0.80, -0.80});
      rclcpp::sleep_for(std::chrono::milliseconds(400));

      RCLCPP_INFO(get_logger(), "[GRIP] Activating continuous squeeze for sphere hold");
      continuous_grip_.store(true);
      std::thread grip_thread(&TaskServerSimple::continuousGripperSqueeze, this);
      grip_thread.detach();

      rclcpp::sleep_for(std::chrono::milliseconds(200));

      RCLCPP_INFO(get_logger(), "Lifting object");

      if (!moveToPose(pregrasp, ee_link))
      {
        RCLCPP_WARN(get_logger(), "Lift-to-pregrasp planning failed - trying Cartesian vertical lift");
        (void)cartesianMoveToZ(pregrasp.pose.position.z, ee_link);
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));

      std::vector<double> carry_joints = {base_angle, -0.3, -0.2};
      RCLCPP_INFO(get_logger(), "Moving to carry position");
      moveToJointPosition(carry_joints);

      rclcpp::sleep_for(std::chrono::milliseconds(200));

      RCLCPP_INFO(get_logger(), "Pick sequence complete - object grasped");

      RCLCPP_INFO(get_logger(), "[GRIP] Deactivating continuous squeeze");
      continuous_grip_.store(false);
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      pick_in_progress_.store(false);
    }

    void performPlace()
    {
      RCLCPP_INFO(get_logger(), "Performing place sequence...");

      std::vector<double> drop_joints = {-1.2, 0.0, -0.2};
      moveToJointPosition(drop_joints);

      moveGripper({-0.7, 0.7});
      rclcpp::sleep_for(std::chrono::seconds(1));

      RCLCPP_INFO(get_logger(), "Place sequence complete");
    }

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
    {
      (void)uuid;
      RCLCPP_INFO(get_logger(), "Received goal request: task_number=%d", goal->task_number);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      (void)goal_handle;
      RCLCPP_INFO(get_logger(), "Received cancel request");
      if (arm_move_group_)
        arm_move_group_->stop();
      if (gripper_move_group_)
        gripper_move_group_->stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      std::thread{std::bind(&TaskServerSimple::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Executing goal");
      auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

      initializeMoveIt();

      int task_num = goal_handle->get_goal()->task_number;

      switch (task_num)
      {
      case 0:
        moveToJointPosition({0.0, 0.0, 0.0});
        moveGripper({-0.7, 0.7});
        break;

      case 1:
        moveToJointPosition({-1.14, -0.6, -0.07});
        break;

      case 2:
        moveToJointPosition({-1.57, 0.0, -0.9});
        break;

      case 3:
        (void)performScan();
        break;

      case 4:
        performPick();
        break;

      case 5:
        performPlace();
        break;

      case 7:
        target_color_ = "red";
        object_detected_.store(false);
        if (performScan())
        {
          RCLCPP_INFO(get_logger(), "Red object detected, stabilizing before pick...");
          rclcpp::sleep_for(std::chrono::seconds(1));
          performPick();
        }
        else
        {
          RCLCPP_WARN(get_logger(), "No red object found during scan");
        }
        break;

      case 8:
        target_color_ = "blue";
        object_detected_.store(false);
        if (performScan())
        {
          RCLCPP_INFO(get_logger(), "Blue object detected, stabilizing before pick...");
          rclcpp::sleep_for(std::chrono::seconds(1));
          performPick();
        }
        else
        {
          RCLCPP_WARN(get_logger(), "No blue object found during scan");
        }
        break;

      case 9:
        target_color_ = "green";
        object_detected_.store(false);
        if (performScan())
        {
          RCLCPP_INFO(get_logger(), "Green object detected, stabilizing before pick...");
          rclcpp::sleep_for(std::chrono::seconds(1));
          performPick();
        }
        else
        {
          RCLCPP_WARN(get_logger(), "No green object found during scan");
        }
        break;

      default:
        RCLCPP_ERROR(get_logger(), "Unknown task number: %d", task_num);
        result->success = false;
        goal_handle->abort(result);
        return;
      }

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
  };

}

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServerSimple)
