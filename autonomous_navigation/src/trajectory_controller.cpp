#include "trajectory_controller.hpp"

// Purpose: Follow a path using RC override commands.
//
// Control structure (decoupled, one PID per DOF, all in the body/yaw frame):
//
//   position error (map, ENU)  --R(yaw)^T-->  e_fwd, e_lat      -> PID -> forward / lateral effort
//   depth error (map z)                                          -> PID -> throttle effort
//   heading error wrap(yaw_des - yaw)                            -> PID -> yaw effort
//
// The derivative term uses the velocity measured by the odometry (EKF) instead of
// differentiating the position error. Efforts are normalized to [-1, 1] and mapped
// to RC PWM with explicit compensation of the ArduSub RC dead-zones.
//
// Note on the plant (ArduSub ALT_HOLD):
//   * forward / lateral sticks  -> thrust           (position loop needs P + D, small I)
//   * throttle stick            -> climb-rate set-point, depth held when centered
//   * yaw stick                 -> yaw-rate set-point, heading held when centered
//   so depth and yaw are outer loops around ArduSub's own rate/hold controllers and
//   usually need only a P (+ small I) gain.

using namespace std::chrono_literals;

namespace
{
constexpr uint16_t kNeutralPwm = 1500;

nav_ctrl::PidGains declareGains(rclcpp::Node & node, const std::string & axis, const nav_ctrl::PidGains & def)
{
    nav_ctrl::PidGains g;
    g.kp = node.declare_parameter<double>("pid." + axis + ".kp", def.kp);
    g.ki = node.declare_parameter<double>("pid." + axis + ".ki", def.ki);
    g.kd = node.declare_parameter<double>("pid." + axis + ".kd", def.kd);
    g.i_max = node.declare_parameter<double>("pid." + axis + ".i_max", def.i_max);
    g.out_max = node.declare_parameter<double>("pid." + axis + ".out_max", def.out_max);
    return g;
}

nav_ctrl::PidGains readGains(rclcpp::Node & node, const std::string & axis)
{
    nav_ctrl::PidGains g;
    g.kp = node.get_parameter("pid." + axis + ".kp").as_double();
    g.ki = node.get_parameter("pid." + axis + ".ki").as_double();
    g.kd = node.get_parameter("pid." + axis + ".kd").as_double();
    g.i_max = node.get_parameter("pid." + axis + ".i_max").as_double();
    g.out_max = node.get_parameter("pid." + axis + ".out_max").as_double();
    return g;
}
}  // namespace

TrajectoryController::TrajectoryController() : Node("trajectory_controller")
{
    // ----------------------------------------------------------- parameters
    this->declare_parameter<std::string>("frame_id", "map");

    // path generator request
    this->declare_parameter<double>("delta", 0.2);
    this->declare_parameter<double>("side", 2.0);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<double>("dz", -1.0);
    this->declare_parameter<int>("turns", 4);
    this->declare_parameter<int>("depth", 4);
    this->declare_parameter<int>("points", 50);
    this->declare_parameter<int>("path_type", 0);

    this->declare_parameter<double>("duct_x", 3.0);
    this->declare_parameter<double>("duct_y", 1.5);
    this->declare_parameter<double>("duct_z", -1.0);

    // PID gains. Efforts are normalized: 1.0 == full stick. Starting points were
    // derived from the previous hard-coded gains (old_gain * 120 / 370).
    declareGains(*this, "forward", {0.8, 0.10, 0.30, 0.30, 0.60});
    declareGains(*this, "lateral", {1.0, 0.10, 0.30, 0.30, 0.60});
    declareGains(*this, "depth",   {1.0, 0.05, 0.00, 0.20, 0.60});
    declareGains(*this, "yaw",     {0.3, 0.00, 0.00, 0.10, 0.40});

    this->declare_parameter<double>("control_rate_hz", 10.0);
    this->declare_parameter<double>("accept_radius", 0.3);
    this->declare_parameter<double>("odom_timeout", 0.5);
    this->declare_parameter<double>("effort_eps", 0.02);
    this->declare_parameter<int>("max_pwm_delta", 400);
    // ArduSub defaults: RC5_DZ/RC6_DZ = 30, RC4_DZ = 40, THR_DZ = 100 (ALT_HOLD climb rate)
    this->declare_parameter<int>("deadzone.forward", 30);
    this->declare_parameter<int>("deadzone.lateral", 30);
    this->declare_parameter<int>("deadzone.yaw", 40);
    this->declare_parameter<int>("deadzone.throttle", 100);
    this->declare_parameter<bool>("twist_in_body_frame", true);
    this->declare_parameter<bool>("hold_final_position", true);
    // 0 = hold the yaw measured when the trajectory started, 1 = look at the duct, 2 = yaw_fixed
    this->declare_parameter<int>("yaw_mode", 0);
    this->declare_parameter<double>("yaw_fixed", 0.0);

    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<std::string>("log_dir", "/home/jetson/Documents/logs_auv");

    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("duct_x", duct_position_.pdx);
    this->get_parameter("duct_y", duct_position_.pdy);
    this->get_parameter("duct_z", duct_position_.pdz);
    loadControllerParams();

    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    if (debug_mode_) {
        const std::string dir = this->get_parameter("log_dir").as_string();
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled, logging to %s", dir.c_str());
        slam_file_.open(dir + "/slam_pose.txt");
        odom_file_.open(dir + "/odometry.txt");
        wp_file_.open(dir + "/wp_file_.txt");
        alt_file_.open(dir + "/alt_file_.txt");
        control_.open(dir + "/control_.txt");
        time_.open(dir + "/time_.txt");
    }

    // ----------------------------------------------------------- publishers
    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("transformed_path", 10);
    effort_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("control/effort", 10);
    error_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("control/error", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("control/target", 10);
    terms_fwd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("control/pid_terms/forward", 10);
    terms_lat_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("control/pid_terms/lateral", 10);
    terms_depth_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("control/pid_terms/depth", 10);
    terms_yaw_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("control/pid_terms/yaw", 10);

    // ------------------------------------------------------ callback groups
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    generate_trajectory_client = this->create_client<autonomous_navigation_msgs::srv::PathGenerator>(
        "generate_path", rmw_qos_profile_services_default, cb_group_);
    mavros_arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming", rmw_qos_profile_services_default, cb_group_);
    mavros_set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode", rmw_qos_profile_services_default, cb_group_);
    start_trajectory_srv = this->create_service<std_srvs::srv::SetBool>(
        "start_trajectory",
        std::bind(&TrajectoryController::start_trajectory_cb, this, std::placeholders::_1, std::placeholders::_2));

    // ---------------------------------------------------------- subscribers
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_sub_;

    slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10, std::bind(&TrajectoryController::slam_cb, this, std::placeholders::_1),
        sub_options);
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, std::bind(&TrajectoryController::state_cb, this, std::placeholders::_1), sub_options);
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "local_path", 10, std::bind(&TrajectoryController::transformPath, this, std::placeholders::_1), sub_options);
    alt_sub_ = this->create_subscription<mavros_msgs::msg::VfrHud>(
        "/mavros/vfr_hud", 10, std::bind(&TrajectoryController::alt_cb, this, std::placeholders::_1), sub_options);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", qos, std::bind(&TrajectoryController::odom_cb, this, std::placeholders::_1),
        sub_options);

    // ---------------------------------------------------------- control loop
    const double rate = std::max(1.0, this->get_parameter("control_rate_hz").as_double());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));
    timer_ = this->create_wall_timer(period, std::bind(&TrajectoryController::pathFollower, this), cb_group_timer_);

    // Live re-tuning: `ros2 param set /navigation/trajectory_controller pid.forward.kp 1.2`
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TrajectoryController::on_parameters_set, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Trajectory controller ready (%.1f Hz)", rate);
}

TrajectoryController::~TrajectoryController()
{
    // Best effort: release the sticks so ArduSub goes back to neutral / pilot input.
    // (A service call from a destructor is not reliable: the executor is already gone.)
    try {
        publish_neutral();
    } catch (...) {
    }
    if (debug_mode_) {
        slam_file_.close();
        odom_file_.close();
        wp_file_.close();
        alt_file_.close();
        control_.close();
        time_.close();
    }
}

// =============================================================== parameters

void TrajectoryController::loadControllerParams()
{
    pid_fwd_.setGains(readGains(*this, "forward"));
    pid_lat_.setGains(readGains(*this, "lateral"));
    pid_depth_.setGains(readGains(*this, "depth"));
    pid_yaw_.setGains(readGains(*this, "yaw"));

    accept_radius_ = this->get_parameter("accept_radius").as_double();
    odom_timeout_ = this->get_parameter("odom_timeout").as_double();
    effort_eps_ = this->get_parameter("effort_eps").as_double();
    max_pwm_delta_ = this->get_parameter("max_pwm_delta").as_int();
    dz_forward_ = this->get_parameter("deadzone.forward").as_int();
    dz_lateral_ = this->get_parameter("deadzone.lateral").as_int();
    dz_yaw_ = this->get_parameter("deadzone.yaw").as_int();
    dz_throttle_ = this->get_parameter("deadzone.throttle").as_int();
    twist_in_body_frame_ = this->get_parameter("twist_in_body_frame").as_bool();
    hold_final_position_ = this->get_parameter("hold_final_position").as_bool();
    yaw_mode_ = static_cast<YawMode>(this->get_parameter("yaw_mode").as_int());
    yaw_fixed_ = this->get_parameter("yaw_fixed").as_double();

    // the integrator is frozen for steps outside this window (e.g. first tick after arming)
    const double max_dt = 5.0 / std::max(1.0, this->get_parameter("control_rate_hz").as_double());
    for (auto * pid : {&pid_fwd_, &pid_lat_, &pid_depth_, &pid_yaw_}) {
        pid->setDtLimits(1e-4, max_dt);
    }
}

rcl_interfaces::msg::SetParametersResult TrajectoryController::on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Validate; the values are applied (re-read) after the callback by the timer thread.
    for (const auto & p : params) {
        const auto & n = p.get_name();
        if (n.rfind("pid.", 0) == 0 && p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = n + " must be a double";
            return result;
        }
        if (n.rfind("pid.", 0) == 0 && (n.find(".kp") != std::string::npos || n.find(".ki") != std::string::npos ||
                                         n.find(".kd") != std::string::npos) && p.as_double() < 0.0) {
            result.successful = false;
            result.reason = n + " must be >= 0";
            return result;
        }
        if (n == "yaw_mode" && (p.as_int() < 0 || p.as_int() > 2)) {
            result.successful = false;
            result.reason = "yaw_mode must be 0 (hold initial), 1 (look at duct) or 2 (fixed)";
            return result;
        }
    }
    // Defer the actual reload to the control thread: get_parameter() inside this
    // callback would still return the *old* values.
    params_dirty_ = true;
    return result;
}

void TrajectoryController::resetControllers()
{
    pid_fwd_.reset();
    pid_lat_.reset();
    pid_depth_.reset();
    pid_yaw_.reset();
    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

// ================================================================= service

void TrajectoryController::start_trajectory_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!request->data) {
        RCLCPP_INFO(this->get_logger(), "Stopping trajectory following");
        follow_path = false;
        publish_neutral();
        response->success = true;
        response->message = "Trajectory following stopped";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting trajectory following");
    follow_path = false;  // make sure the loop is idle while we re-initialise

    // 1) Re-capture the origin from the *next* odometry sample and wait for it, so that
    //    the path (expressed relative to the start pose) is transformed with the fresh origin.
    set_origin_ = false;
    const auto t_origin = std::chrono::steady_clock::now();
    while (!set_origin_ && rclcpp::ok()) {
        if (std::chrono::steady_clock::now() - t_origin > 2s) {
            response->success = false;
            response->message = "No odometry received, cannot set the trajectory origin";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        rclcpp::sleep_for(20ms);
    }

    // 2) Ask the generator for a new path
    auto req = std::make_shared<autonomous_navigation_msgs::srv::PathGenerator::Request>();
    req->delta = this->get_parameter("delta").as_double();
    req->side = this->get_parameter("side").as_double();
    req->radius = this->get_parameter("radius").as_double();
    req->dz = this->get_parameter("dz").as_double();
    req->turns = this->get_parameter("turns").as_int();
    req->depth = this->get_parameter("depth").as_int();
    req->points = this->get_parameter("points").as_int();
    req->path_type = this->get_parameter("path_type").as_int();

    if (!generate_trajectory_client->wait_for_service(1s)) {
        response->success = false;
        response->message = "generate_path service not available";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    path_received_ = false;
    auto future = generate_trajectory_client->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
        response->success = false;
        response->message = "Timeout waiting for generate_path";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    auto result = future.get();
    if (!result->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate trajectory: %s", result->message.c_str());
        response->success = false;
        response->message = "Failed to generate trajectory: " + result->message;
        return;
    }

    // 3) Wait for the path message instead of sleeping a fixed 2 s
    const auto t_path = std::chrono::steady_clock::now();
    while (!path_received_ && rclcpp::ok()) {
        if (std::chrono::steady_clock::now() - t_path > 3s) {
            response->success = false;
            response->message = "Path generated but never received on local_path";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        rclcpp::sleep_for(20ms);
    }

    // 4) Bumpless start
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        trajectory_index_ = 0;
    }
    resetControllers();
    finished_ = false;
    follow_path = true;

    RCLCPP_INFO(this->get_logger(), "Trajectory generated successfully, following started");
    response->success = true;
    response->message = "Trajectory generated and started successfully";
}

// =============================================================== FCU state

void TrajectoryController::state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg)
{
    connected_ = msg->connected;
    armed_ = msg->armed;
    std::lock_guard<std::mutex> lk(mode_mutex_);
    mode_ = msg->mode;
}

bool TrajectoryController::set_arm(bool arm)
{
    if (!connected_) {
        RCLCPP_ERROR(this->get_logger(), "ROV not connected, cannot change arm state");
        return false;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    if (mavros_arm_client_->service_is_ready()) {
        mavros_arm_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Requested %s", arm ? "ARM" : "DISARM");
        return true;
    }
    return false;
}

bool TrajectoryController::set_mode(const std::string & mode)
{
    if (!connected_) {
        RCLCPP_ERROR(this->get_logger(), "ROV not connected, cannot change mode");
        return false;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    if (mavros_set_mode_client_->service_is_ready()) {
        mavros_set_mode_client_->async_send_request(request);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Requested mode %s", mode.c_str());
        return true;
    }
    return false;
}

// ================================================================ RC output

void TrajectoryController::publish_rc(double forward, double lateral, double depth, double yaw)
{
    mavros_msgs::msg::OverrideRCIn rc_msg;
    for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
    }
    // ROS body frame (FLU, ENU yaw) -> ArduSub sticks:
    //   forward: 1900 = forward            (same sign)
    //   lateral: 1900 = right  -> ROS +y is left  (reverse)
    //   yaw:     1900 = clockwise -> ROS +yaw is CCW (reverse)
    //   throttle:1900 = up                 (same sign)
    rc_msg.channels[5 - 1] = nav_ctrl::effortToPwm(forward, false, dz_forward_, max_pwm_delta_, effort_eps_);
    rc_msg.channels[6 - 1] = nav_ctrl::effortToPwm(lateral, true, dz_lateral_, max_pwm_delta_, effort_eps_);
    rc_msg.channels[4 - 1] = nav_ctrl::effortToPwm(yaw, true, dz_yaw_, max_pwm_delta_, effort_eps_);
    rc_msg.channels[3 - 1] = nav_ctrl::effortToPwm(depth, false, dz_throttle_, max_pwm_delta_, effort_eps_);

    rc_pub_->publish(rc_msg);
}

void TrajectoryController::publish_neutral()
{
    mavros_msgs::msg::OverrideRCIn rc_msg;
    for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
    }
    rc_msg.channels[5 - 1] = kNeutralPwm;
    rc_msg.channels[6 - 1] = kNeutralPwm;
    rc_msg.channels[4 - 1] = kNeutralPwm;
    rc_msg.channels[3 - 1] = kNeutralPwm;
    rc_pub_->publish(rc_msg);
}

// ================================================================== inputs

void TrajectoryController::publishPath()
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = frame_id_;
    for (const auto & p : path_transformed_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = p.z;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
}

void TrajectoryController::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
    std::lock_guard<std::mutex> lk(data_mutex_);

    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    current_vel_ = msg->twist.twist;
    last_odom_time_ = this->now();

    if (!set_origin_) {
        origin_x_ = current_pose_.pose.position.x;
        origin_y_ = current_pose_.pose.position.y;
        origin_z_ = current_pose_.pose.position.z;
        origin_yaw_ = tf2::getYaw(current_pose_.pose.orientation);
        set_origin_ = true;
        RCLCPP_INFO(get_logger(), "Origin set: x=%.2f y=%.2f z=%.2f yaw=%.1f deg", origin_x_, origin_y_, origin_z_,
                    origin_yaw_ * 180.0 / M_PI);
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "odom frame_id: %s, child_frame_id: %s", msg->header.frame_id.c_str(),
                     msg->child_frame_id.c_str());

    if (debug_mode_) {
        odom_file_ << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " "
                   << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " "
                   << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " "
                   << msg->pose.pose.orientation.w << '\n';
    }
}

void TrajectoryController::slam_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
    if (debug_mode_) {
        slam_file_ << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z << " "
                   << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z
                   << " " << msg->pose.orientation.w << '\n';
    }
}

void TrajectoryController::alt_cb(const mavros_msgs::msg::VfrHud::ConstSharedPtr & msg)
{
    if (debug_mode_) {
        alt_file_ << msg->altitude << '\n';
    }
}

void TrajectoryController::transformPath(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
    std::lock_guard<std::mutex> lk(data_mutex_);

    // NOTE: the path is translated to the start position but NOT rotated by the
    // initial yaw (the previous behaviour is kept). If you want the shape to be
    // expressed in the ROV's initial heading, rotate (x, y) by origin_yaw_ here.
    path_transformed_.clear();
    path_transformed_.reserve(msg->poses.size());
    for (const auto & p : msg->poses) {
        path_transformed_.push_back({p.pose.position.x + origin_x_, p.pose.position.y + origin_y_,
                                     p.pose.position.z + origin_z_});
    }

    if (!path_transformed_.empty()) {
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto & p : path_transformed_) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        const double n = static_cast<double>(path_transformed_.size());
        duct_position_.pdx = sum_x / n;
        duct_position_.pdy = sum_y / n;
        duct_position_.pdz = sum_z / n;
        RCLCPP_INFO(this->get_logger(), "Path with %zu points. Center / duct position: [%.3f, %.3f, %.3f]",
                    path_transformed_.size(), duct_position_.pdx, duct_position_.pdy, duct_position_.pdz);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received path is empty. Duct position was not updated.");
    }

    if (debug_mode_) {
        for (const auto & p : path_transformed_) {
            wp_file_ << p.x << " " << p.y << " " << p.z << "\n";
        }
        wp_file_ << std::endl;
    }
    trajectory_index_ = 0;
    path_received_ = !path_transformed_.empty();
    publishPath();
}

// ================================================================= heading

tf2::Quaternion TrajectoryController::lookAtTheDuct(const geometry_msgs::msg::Point & robot_position)
{
    // Gram-Schmidt: body x axis points to the duct, z stays as vertical as possible
    Eigen::Vector3d pduct(duct_position_.pdx, duct_position_.pdy, duct_position_.pdz);
    Eigen::Vector3d robot(robot_position.x, robot_position.y, robot_position.z);

    Eigen::Vector3d x_axis = pduct - robot;
    if (x_axis.norm() < 1e-6) {
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    }
    x_axis.normalize();

    Eigen::Vector3d z_ref(0, 0, 1);
    Eigen::Vector3d z_axis = z_ref - z_ref.dot(x_axis) * x_axis;
    if (z_axis.norm() < 1e-3) {
        z_ref = Eigen::Vector3d(0, 1, 0);
        z_axis = z_ref - z_ref.dot(x_axis) * x_axis;
    }
    z_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    Eigen::Quaterniond q(R);
    tf2::Quaternion q_tf;
    q_tf.setValue(q.x(), q.y(), q.z(), q.w());
    return q_tf;
}

double TrajectoryController::desiredYaw(const geometry_msgs::msg::Point & robot_position) const
{
    switch (yaw_mode_) {
        case YawMode::LOOK_AT_DUCT:
            // planar look-at is all we need for the yaw DOF
            return std::atan2(duct_position_.pdy - robot_position.y, duct_position_.pdx - robot_position.x);
        case YawMode::FIXED:
            return yaw_fixed_;
        case YawMode::HOLD_INITIAL:
        default:
            return origin_yaw_;
    }
}

// =============================================================== debug out

void TrajectoryController::publishDebug(const rclcpp::Time & stamp, const waypoint & target, double e_fwd,
                                        double e_lat, double e_z, double e_yaw)
{
    geometry_msgs::msg::TwistStamped eff;
    eff.header.stamp = stamp;
    eff.header.frame_id = "base_link";
    eff.twist.linear.x = pid_fwd_.terms().out;
    eff.twist.linear.y = pid_lat_.terms().out;
    eff.twist.linear.z = pid_depth_.terms().out;
    eff.twist.angular.z = pid_yaw_.terms().out;
    effort_pub_->publish(eff);

    geometry_msgs::msg::TwistStamped err;
    err.header = eff.header;
    err.twist.linear.x = e_fwd;
    err.twist.linear.y = e_lat;
    err.twist.linear.z = e_z;
    err.twist.angular.z = e_yaw;
    error_pub_->publish(err);

    geometry_msgs::msg::PointStamped tgt;
    tgt.header.stamp = stamp;
    tgt.header.frame_id = frame_id_;
    tgt.point.x = target.x;
    tgt.point.y = target.y;
    tgt.point.z = target.z;
    target_pub_->publish(tgt);

    auto terms_msg = [&](const nav_ctrl::AxisPid & pid) {
        geometry_msgs::msg::Vector3Stamped m;
        m.header = eff.header;
        m.vector.x = pid.terms().p;
        m.vector.y = pid.terms().i;
        m.vector.z = pid.terms().d;
        return m;
    };
    terms_fwd_pub_->publish(terms_msg(pid_fwd_));
    terms_lat_pub_->publish(terms_msg(pid_lat_));
    terms_depth_pub_->publish(terms_msg(pid_depth_));
    terms_yaw_pub_->publish(terms_msg(pid_yaw_));

    if (debug_mode_) {
        control_ << pid_fwd_.terms().out << " " << pid_lat_.terms().out << " " << pid_depth_.terms().out << " "
                 << pid_yaw_.terms().out << "\n";
        time_ << stamp.seconds() << "\n";
    }
}

// ============================================================ control loop

void TrajectoryController::pathFollower()
{
    if (params_dirty_.exchange(false)) {
        loadControllerParams();
        RCLCPP_INFO(this->get_logger(), "Controller parameters reloaded");
    }

    if (!follow_path) {
        return;
    }

    // ------------------------------------------------ FCU pre-conditions
    if (!connected_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ROV not connected");
        resetControllers();
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mode_mutex_);
        if (mode_ != "ALT_HOLD") {
            set_mode("ALT_HOLD");
            resetControllers();
            return;
        }
    }
    if (!armed_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ROV not armed, arming now");
        set_arm(true);
        resetControllers();
        return;
    }

    // ------------------------------------------- snapshot of shared state
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist vel;
    waypoint target{};
    double yaw_des = 0.0;
    bool last_wp = false;
    rclcpp::Time now = this->now();
    {
        std::lock_guard<std::mutex> lk(data_mutex_);

        if (path_transformed_.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Trajectory is empty");
            return;
        }
        if (last_odom_time_.nanoseconds() == 0 || (now - last_odom_time_).seconds() > odom_timeout_) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Odometry timeout (> %.2f s) - sending neutral", odom_timeout_);
            publish_neutral();
            resetControllers();
            return;
        }

        pose = current_pose_.pose;
        vel = current_vel_;

        // Advance through every waypoint already inside the acceptance radius
        // (the path is densely sampled; advancing one index per tick limited the speed).
        const auto dist_to = [&](const waypoint & w) {
            return std::hypot(w.x - pose.position.x, w.y - pose.position.y, w.z - pose.position.z);
        };
        while (trajectory_index_ + 1 < path_transformed_.size() &&
               dist_to(path_transformed_[trajectory_index_]) < accept_radius_) {
            ++trajectory_index_;
        }
        last_wp = (trajectory_index_ + 1 >= path_transformed_.size());
        target = path_transformed_[trajectory_index_];
        yaw_des = desiredYaw(pose.position);

        if (last_wp && dist_to(target) < accept_radius_) {
            if (!finished_) {
                finished_ = true;
                RCLCPP_INFO(this->get_logger(), "Trajectory completed%s",
                            hold_final_position_ ? ", holding final position" : "");
            }
            if (!hold_final_position_) {
                publish_neutral();
                follow_path = false;
                return;
            }
        }
    }

    // ----------------------------------------------------------- timing
    if (last_time_.nanoseconds() == 0) {
        last_time_ = now;
        return;  // first tick: only initialise the clock
    }
    const double dt = (now - last_time_).seconds();
    last_time_ = now;

    // ----------------------------------------------------------- errors
    const double yaw = tf2::getYaw(pose.orientation);

    const double ex = target.x - pose.position.x;
    const double ey = target.y - pose.position.y;
    const double ez = target.z - pose.position.z;

    double e_fwd, e_lat;
    nav_ctrl::worldToBody(yaw, ex, ey, e_fwd, e_lat);

    const double e_yaw = nav_ctrl::wrapPi(yaw_des - yaw);

    // -------------------------------------------- measured velocities
    // For a piecewise-constant set-point, d(error)/dt = -velocity.
    double v_fwd, v_lat, v_z;
    if (twist_in_body_frame_) {
        // MAVROS local_position/odom: twist is expressed in base_link (FLU)
        v_fwd = vel.linear.x;
        v_lat = vel.linear.y;
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        v_z = tf2::quatRotate(q, tf2::Vector3(vel.linear.x, vel.linear.y, vel.linear.z)).z();
    } else {
        nav_ctrl::worldToBody(yaw, vel.linear.x, vel.linear.y, v_fwd, v_lat);
        v_z = vel.linear.z;
    }
    const double yaw_rate = vel.angular.z;

    // -------------------------------------------------------------- PID
    const double u_fwd = pid_fwd_.update(e_fwd, -v_fwd, dt);
    const double u_lat = pid_lat_.update(e_lat, -v_lat, dt);
    const double u_depth = pid_depth_.update(ez, -v_z, dt);
    const double u_yaw = pid_yaw_.update(e_yaw, -yaw_rate, dt);

    publish_rc(u_fwd, u_lat, u_depth, u_yaw);
    publishDebug(now, target, e_fwd, e_lat, ez, e_yaw);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryController>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
