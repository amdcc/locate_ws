#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pid_control/pid_control.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double normalize_angle(double angle)
{
	while (angle > M_PI) {
		angle -= 2.0 * M_PI;
	}
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}
	return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
	const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
	geometry_msgs::msg::Quaternion q;
	q.x = 0.0;
	q.y = 0.0;
	q.z = std::sin(yaw * 0.5);
	q.w = std::cos(yaw * 0.5);
	return q;
}
}  // namespace

WaypointManager::WaypointManager()
	: Node("waypoint_manager"), current_index_(0), has_pose_(false)
{
  waypoint_frame_ = declare_parameter<std::string>("waypoint_frame", "map");
  reach_tolerance_ = declare_parameter<double>("reach_tolerance", 0.20);
  loop_waypoints_ = declare_parameter<bool>("loop_waypoints", true);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  waypoint_list_raw_ = declare_parameter<std::vector<double>>(
    "waypoint_list", pid_control::kDefaultWaypointList);

  load_waypoints();

  target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pid_control::kTargetPoseTopic, 10);
  tracked_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pid_control::kTrackedPoseTopic, 20,
    std::bind(&WaypointManager::tracked_pose_callback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&WaypointManager::publish_current_target, this));

  publish_current_target();
  RCLCPP_INFO(get_logger(), "waypoint_manager initialized with %zu waypoints", waypoints_.size());
}

void WaypointManager::load_waypoints()
{
  if (waypoint_list_raw_.size() < 3 || waypoint_list_raw_.size() % 3 != 0) {
    throw std::runtime_error("Parameter waypoint_list must contain x,y,yaw triples.");
  }

  waypoints_.clear();
  for (std::size_t i = 0; i < waypoint_list_raw_.size(); i += 3) {
    waypoints_.push_back({waypoint_list_raw_[i], waypoint_list_raw_[i + 1], waypoint_list_raw_[i + 2]});
  }
}

void WaypointManager::tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  has_pose_ = true;
  maybe_advance_waypoint();
}

void WaypointManager::maybe_advance_waypoint()
{
  if (!has_pose_ || waypoints_.empty()) {
    return;
  }

  const auto & wp = waypoints_[current_index_];
  const double dx = wp.x - current_pose_.pose.position.x;
  const double dy = wp.y - current_pose_.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance > reach_tolerance_) {
    return;
  }

  if (current_index_ + 1 < waypoints_.size()) {
    ++current_index_;
    RCLCPP_INFO(get_logger(), "Reached waypoint, switched to index %zu", current_index_);
  } else if (loop_waypoints_) {
    current_index_ = 0;
    RCLCPP_INFO(get_logger(), "Reached last waypoint, looped to index 0");
  }
  publish_current_target();
}

void WaypointManager::publish_current_target()
{
  if (waypoints_.empty()) {
    return;
  }

  const auto & wp = waypoints_[current_index_];
  geometry_msgs::msg::PoseStamped target;
  target.header.stamp = now();
  target.header.frame_id = waypoint_frame_;
  target.pose.position.x = wp.x;
  target.pose.position.y = wp.y;
  target.pose.position.z = 0.0;
  target.pose.orientation = quaternion_from_yaw(wp.yaw);

  target_pub_->publish(target);
}

PoseProcessor::PoseProcessor()
	: Node("pose_processor"), has_pose_(false), has_target_(false)
{
  error_pub_ = create_publisher<geometry_msgs::msg::Twist>(pid_control::kPoseErrorTopic, 20);

  tracked_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pid_control::kTrackedPoseTopic, 20,
    std::bind(&PoseProcessor::tracked_pose_callback, this, std::placeholders::_1));
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pid_control::kTargetPoseTopic, 20,
    std::bind(&PoseProcessor::target_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "pose_processor initialized");
}

void PoseProcessor::tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  has_pose_ = true;
  compute_and_publish_error();
}

void PoseProcessor::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_pose_ = *msg;
  has_target_ = true;
  compute_and_publish_error();
}

void PoseProcessor::compute_and_publish_error()
{
  if (!has_pose_ || !has_target_) {
    return;
  }

  const double current_yaw = yaw_from_quaternion(current_pose_.pose.orientation);
  const double target_yaw = yaw_from_quaternion(target_pose_.pose.orientation);
  const double dx_world = target_pose_.pose.position.x - current_pose_.pose.position.x;
  const double dy_world = target_pose_.pose.position.y - current_pose_.pose.position.y;

  const double ex_robot = std::cos(current_yaw) * dx_world + std::sin(current_yaw) * dy_world;
  const double ey_robot = -std::sin(current_yaw) * dx_world + std::cos(current_yaw) * dy_world;
  const double heading_error = normalize_angle(target_yaw - current_yaw);

  geometry_msgs::msg::Twist error_msg;
  error_msg.linear.x = ex_robot;
  error_msg.linear.y = ey_robot;
  error_msg.linear.z = std::hypot(dx_world, dy_world);
  error_msg.angular.z = heading_error;

  error_pub_->publish(error_msg);
}

DiffDriveController::DiffDriveController()
	: Node("diff_drive_controller"), prev_lin_err_(0.0), prev_ang_err_(0.0), i_lin_(0.0), i_ang_(0.0),
	  has_error_(false)
{
  kp_linear_ = declare_parameter<double>("kp_linear", 0.8);
  ki_linear_ = declare_parameter<double>("ki_linear", 0.0);
  kd_linear_ = declare_parameter<double>("kd_linear", 0.1);
  kp_angular_ = declare_parameter<double>("kp_angular", 1.6);
  ki_angular_ = declare_parameter<double>("ki_angular", 0.0);
  kd_angular_ = declare_parameter<double>("kd_angular", 0.15);

  cross_track_gain_ = declare_parameter<double>("cross_track_gain", 1.0);
  max_linear_vel_ = declare_parameter<double>("max_linear_vel", 0.7);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.5);
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 30.0);
  wheel_base_ = declare_parameter<double>("wheel_base", 0.38);
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
  publish_wheel_speeds_ = declare_parameter<bool>("publish_wheel_speeds", true);
  wheel_speeds_topic_ = declare_parameter<std::string>("wheel_speeds_topic", pid_control::kWheelSpeedsTopic);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(pid_control::kCmdVelTopic, 20);
  wheel_speed_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_speeds_topic_, 20);

  error_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    pid_control::kPoseErrorTopic, 20,
    std::bind(&DiffDriveController::error_callback, this, std::placeholders::_1));

  prev_time_ = now();
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&DiffDriveController::control_step, this));

  RCLCPP_INFO(get_logger(), "diff_drive_controller initialized");
}

double DiffDriveController::clamp(double value, double min_v, double max_v)
{
  return std::min(std::max(value, min_v), max_v);
}

void DiffDriveController::error_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_error_ = *msg;
  has_error_ = true;
}

void DiffDriveController::control_step()
{
  if (!has_error_) {
    return;
  }

  const rclcpp::Time current_time = now();
  double dt = (current_time - prev_time_).seconds();
  if (dt <= 1e-6) {
    dt = 1e-3;
  }
  prev_time_ = current_time;

  const double ex = last_error_.linear.x;
  const double ey = last_error_.linear.y;
  const double e_theta = last_error_.angular.z;

  i_lin_ += ex * dt;
  i_ang_ += e_theta * dt;

  const double d_lin = (ex - prev_lin_err_) / dt;
  const double d_ang = (e_theta - prev_ang_err_) / dt;

  prev_lin_err_ = ex;
  prev_ang_err_ = e_theta;

  const double v_pid = kp_linear_ * ex + ki_linear_ * i_lin_ + kd_linear_ * d_lin;
  const double w_pid = kp_angular_ * e_theta + ki_angular_ * i_ang_ + kd_angular_ * d_ang;
  const double w_cross_track = cross_track_gain_ * std::atan2(ey, std::max(0.05, std::abs(ex)));

  const double v_cmd = clamp(v_pid, -max_linear_vel_, max_linear_vel_);
  const double w_cmd = clamp(w_pid + w_cross_track, -max_angular_vel_, max_angular_vel_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = v_cmd;
  cmd_vel.angular.z = w_cmd;
  cmd_vel_pub_->publish(cmd_vel);

  if (publish_wheel_speeds_) {
    const double left_rad_s = (2.0 * v_cmd - w_cmd * wheel_base_) / (2.0 * wheel_radius_);
    const double right_rad_s = (2.0 * v_cmd + w_cmd * wheel_base_) / (2.0 * wheel_radius_);

    const double left_rps = left_rad_s / kTwoPi;
    const double right_rps = right_rad_s / kTwoPi;

    std_msgs::msg::Float64MultiArray wheel_msg;
    // data[0] = left wheel speed (rps), data[1] = right wheel speed (rps).
    wheel_msg.data = {left_rps, right_rps};
    wheel_speed_pub_->publish(wheel_msg);
  }
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	auto waypoint_manager = std::make_shared<WaypointManager>();
	auto pose_processor = std::make_shared<PoseProcessor>();
	auto diff_drive_controller = std::make_shared<DiffDriveController>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(waypoint_manager);
	executor.add_node(pose_processor);
	executor.add_node(diff_drive_controller);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}

