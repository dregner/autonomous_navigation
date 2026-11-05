#ifndef TRAJECTORY_CONTROLLER_HPP
#define TRAJECTORY_CONTROLLER_HPP

#include <atomic>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <autonomous_navigation_msgs/srv/path_generator.hpp>
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <Eigen/Dense>

#include "pid.hpp"

class TrajectoryController : public rclcpp::Node
{
public:
    TrajectoryController();
    ~TrajectoryController();

private:
    // ------------------------------------------------------------------ types
    struct waypoint
    {
        double x;
        double y;
        double z;
    };

    struct DuctPosition
    {
        double pdx{3.0};
        double pdy{1.5};
        double pdz{-1.0};
    };

    enum class YawMode { HOLD_INITIAL = 0, LOOK_AT_DUCT = 1, FIXED = 2 };

    // -------------------------------------------------------------- callbacks
    void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
    void state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg);
    void transformPath(const nav_msgs::msg::Path::ConstSharedPtr & msg);
    void alt_cb(const mavros_msgs::msg::VfrHud::ConstSharedPtr & msg);
    void slam_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);
    void start_trajectory_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> & params);

    // ---------------------------------------------------------------- helpers
    bool set_arm(bool arm);
    bool set_mode(const std::string & mode);
    void publish_rc(double forward, double lateral, double depth, double yaw);
    void publish_neutral();
    void publishPath();
    void publishDebug(const rclcpp::Time & stamp, const waypoint & target,
                      double e_fwd, double e_lat, double e_z, double e_yaw);
    void pathFollower();
    void loadControllerParams();
    void resetControllers();
    tf2::Quaternion lookAtTheDuct(const geometry_msgs::msg::Point & robot_position);
    double desiredYaw(const geometry_msgs::msg::Point & robot_position, double yaw) const;

    // ----------------------------------------------------------- ROS handles
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr alt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_trajectory_srv;

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // Debug / tuning topics (plot with rqt_plot or PlotJuggler)
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr effort_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr terms_fwd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr terms_lat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr terms_depth_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr terms_yaw_pub_;

    rclcpp::Client<autonomous_navigation_msgs::srv::PathGenerator>::SharedPtr generate_trajectory_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr mavros_arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mavros_set_mode_client_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ------------------------------------------------------------ FCU state
    std::atomic<bool> connected_{false};
    std::atomic<bool> armed_{false};
    std::string mode_;
    std::mutex mode_mutex_;

    // ---------------------------------------------------- shared robot state
    // Everything below is written by subscriber callbacks and read by the
    // control timer (different callback groups in a MultiThreadedExecutor)
    // -> guarded by data_mutex_.
    std::mutex data_mutex_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_vel_;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    std::vector<waypoint> path_transformed_;
    size_t trajectory_index_{0};
    double origin_x_{0.0};
    double origin_y_{0.0};
    double origin_z_{0.0};
    double origin_yaw_{0.0};
    DuctPosition duct_position_;

    std::atomic<bool> set_origin_{false};
    std::atomic<bool> path_received_{false};
    std::atomic<bool> follow_path{false};
    std::atomic<bool> finished_{false};
    std::atomic<bool> params_dirty_{false};

    // ------------------------------------------------------------ controller
    nav_ctrl::AxisPid pid_fwd_;
    nav_ctrl::AxisPid pid_lat_;
    nav_ctrl::AxisPid pid_depth_;
    nav_ctrl::AxisPid pid_yaw_;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

    // tunables (ROS parameters, see loadControllerParams())
    double accept_radius_{0.3};
    double odom_timeout_{0.5};
    double effort_eps_{0.02};
    double yaw_effort_eps_{0.005};
    double max_horizontal_speed_{0.0};
    int max_pwm_delta_{400};
    int dz_forward_{30};
    int dz_lateral_{30};
    int dz_yaw_{40};
    int dz_throttle_{100};
    bool twist_in_body_frame_{true};
    bool hold_final_position_{true};
    YawMode yaw_mode_{YawMode::HOLD_INITIAL};
    double yaw_fixed_{0.0};

    std::string frame_id_;

    // ----------------------------------------------------------------- logs
    std::ofstream slam_file_;
    std::ofstream odom_file_;
    std::ofstream wp_file_;
    std::ofstream alt_file_;
    std::ofstream control_;
    std::ofstream time_;
    bool debug_mode_{false};
};

#endif  // TRAJECTORY_CONTROLLER_HPP
