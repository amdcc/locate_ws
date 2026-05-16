#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace pid_control
{
struct Waypoint
{
	double x;
	double y;
	double yaw;
};

inline const std::vector<Waypoint> kFixedWaypoints = {
	{0.0, 0.0, 0.0},
	{1.5, 0.0, 0.0},
	{2.0, 1.0, 1.57},
	{1.0, 2.0, 3.14},
	{0.0, 1.0, -1.57},
};

inline constexpr const char * kTrackedPoseTopic = "/tracked_pose";
inline constexpr const char * kTargetPoseTopic = "/target_pose";
inline constexpr const char * kPoseErrorTopic = "/pose_error";
inline constexpr const char * kCmdVelTopic = "/cmd_vel";
inline constexpr const char * kWheelSpeedsTopic = "/wheel_speeds";

inline const std::vector<double> kDefaultWaypointList = {
	0.0, 0.0, 0.0,
	1.5, 0.0, 0.0,
	2.0, 1.0, 1.57,
	1.0, 2.0, 3.14,
	0.0, 1.0, -1.57,
};
}  // namespace pid_control

class WaypointManager : public rclcpp::Node
{
public:
	WaypointManager();

private:
	struct Waypoint
	{
		double x;
		double y;
		double yaw;
	};

	void load_waypoints();
	void tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void maybe_advance_waypoint();
	void publish_current_target();

	std::string waypoint_frame_;
	double reach_tolerance_;
	bool loop_waypoints_;
	double publish_rate_hz_;
	std::vector<double> waypoint_list_raw_;
	std::vector<Waypoint> waypoints_;
	std::size_t current_index_;
	geometry_msgs::msg::PoseStamped current_pose_;
	bool has_pose_;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

class PoseProcessor : public rclcpp::Node
{
public:
	PoseProcessor();

private:
	void tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void compute_and_publish_error();

	geometry_msgs::msg::PoseStamped current_pose_;
	geometry_msgs::msg::PoseStamped target_pose_;
	bool has_pose_;
	bool has_target_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr error_pub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
};

class DiffDriveController : public rclcpp::Node
{
public:
	DiffDriveController();

private:
	static double clamp(double value, double min_v, double max_v);
	void error_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void control_step();

	double kp_linear_;
	double ki_linear_;
	double kd_linear_;
	double kp_angular_;
	double ki_angular_;
	double kd_angular_;
	double cross_track_gain_;
	double max_linear_vel_;
	double max_angular_vel_;
	double control_rate_hz_;
	double wheel_base_;
	double wheel_radius_;
	bool publish_wheel_speeds_;
	std::string wheel_speeds_topic_;

	double prev_lin_err_;
	double prev_ang_err_;
	double i_lin_;
	double i_ang_;
	rclcpp::Time prev_time_;

	bool has_error_;
	geometry_msgs::msg::Twist last_error_;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_pub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr error_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};
