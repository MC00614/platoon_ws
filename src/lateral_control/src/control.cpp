#include "control.h"

Control::Control(int truck_id) : rclcpp::Node("truck" + std::to_string(truck_id) + "_lateral_control"), truck_id(truck_id) {
    // Stanley
    double k = 1.5;
    double ks = 10.2;

    this->controller = Stanley(k, ks);

    // PID
    double Kp = 2.0;
    double Ki = 5.0;
    double Kd = 1.0;

    this->current_yaw = deg2rad(90.0);

    this->pid = new PID(&(this->current_yaw), &(this->steerCommand), &(this->setpoint), Kp, Ki, Kd, _PID_P_ON_E, _PID_CD_DIRECT);

    // this->pid->SetMode(_PID_MODE_AUTOMATIC);

    this->pathValid = false;
    this->velocityValid = false;

    this->window_height = 480.0;
    this->window_width = 640.0;

    // Subscribe
    std::string topic_name;
    topic_name = "/platoon/truck" + std::to_string(truck_id) + "/path";
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        topic_name, 10, std::bind(&Control::path_callback, this,  std::placeholders::_1));

    topic_name = "/truck" + std::to_string(truck_id) + "/velocity";
    velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        topic_name, 10, std::bind(&Control::velocity_callback, this, std::placeholders::_1));

    // Publish
    topic_name = "/truck" + std::to_string(truck_id) + "/steer_control";
    steer_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        topic_name, 10);

    publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Control::publisher_timer_callback, this)
    );
}

void Control::path_callback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    this->pathValid = false;
    this->refPoses.clear();
    Path refPose;
    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
        refPose.x = path_msg->poses[i].pose.position.x;
        refPose.y = path_msg->poses[i].pose.position.y;
        // refPose.yaw = this->quat_to_yaw(path_msg->poses[i].pose.orientation);
        // std::cout << "Raw Lane Info X : " << refPose.x << " Y : " << refPose.y << std::endl;
        this->refPoses.push_back(refPose);
    }
    this->pathValid = true;
}

void Control::velocity_callback(const std_msgs::msg::Float32::SharedPtr velocity_msg) {
    this->velocityValid = false;
    this->current_velocity = velocity_msg->data;
    this->velocityValid = true;
}

void Control::publisher_timer_callback() {
    if (!this->pathValid) { std::cout << "Path Message Receive Error" << std::endl; return;}

    std::vector<Path> relative_middle;
    if (!this->refPoses.empty()) {
        relative_middle = extract_target_point();
        this->setpoint = relative_middle[0].yaw;  
    }
    else {
        std::cout << "No Lane" << std::endl;
        return;
    }

    // stanley
    this->controller.stanley_control(relative_middle, this->current_velocity);
    this->steerCommand = this->controller.getDelta();

    // pid
    // this->pid->Compute();

    float normalize_steer = normalize_steer_command(30.0);

    this->publish_steer(normalize_steer);
}

void Control::publish_steer(float steer) {
    auto steer_msg = std::make_unique<std_msgs::msg::Float32>();
    steer_msg->data = steer;
    // std::cout << " Publish Steering : " << steer << std::endl;
    this->steer_publisher_->publish(std::move(steer_msg));
}

float Control::quat_to_yaw(const geometry_msgs::msg::Quaternion quat) {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3 matrix(tf2_quat);
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

float Control::normalize_steer_command(float max_steer_deg) {
    // Normalize the steer to the range -30 to 30

    float steer_deg = (rad2deg(this->steerCommand) * (-1) + 90.0);

    if (steer_deg > max_steer_deg) {steer_deg = max_steer_deg;}
    else if (steer_deg < max_steer_deg * (-1)) {steer_deg = max_steer_deg * (-1);}
    std::cout << "Last Steer Command : " << steer_deg <<std::endl;

    steer_deg *= -1;

    return steer_deg;
}

std::vector<Path> Control::extract_target_point() {
    std::vector<Path> relative_middle;
    Path relative_path;

    // Check if refPoses has at least two elements to avoid out of range access
    if (this->refPoses.size() == 1) {
        float pose_x = this->window_width / 2.0;
        float pose_y = this->window_height;

        float pose_x1 = refPoses[0].x;
        float pose_y1 = refPoses[0].y;

        float relative_x = this->window_width - pose_x;
        float relative_y = this->window_height - pose_y;
        float relative_x1 = this->window_width - pose_x1;
        float relative_y1 = this->window_height - pose_y1;

        float yaw = M_PI - atan2(relative_y1 - relative_y, relative_x1 - relative_x);

        relative_path.x = relative_x1;
        relative_path.y = relative_y1 * 10;
        relative_path.yaw = yaw;
        relative_middle.push_back(relative_path);

        return relative_middle;
    }

    for (size_t i = 0; i < this->refPoses.size() - 1; ++i) {
        float pose_x1 = refPoses[i].x;
        float pose_y1 = refPoses[i].y;

        float pose_x2 = refPoses[i + 1].x;
        float pose_y2 = refPoses[i + 1].y;

        // Skip invalid points
        // if (pose_x1 < 0 || pose_y1 < 0 || pose_x2 < 0 || pose_y2 < 0) {
        //     continue;
        // }
        // if (pose_x1 > this->window_width || pose_x2 > this->window_width) {
        //     continue;
        // }
        // if (pose_y1 > this->window_height || pose_y2 > this->window_height) {
        //     continue;
        // }

        float relative_x1 = this->window_width - pose_x1;
        float relative_y1 = this->window_height - pose_y1;
        float relative_x2 = this->window_width - pose_x2;
        float relative_y2 = this->window_height - pose_y2;

        float yaw = M_PI - atan2(relative_y2 - relative_y1, relative_x2 - relative_x1);

        relative_path.x = relative_x1;
        relative_path.y = relative_y1 * 10;
        relative_path.yaw = yaw;
        relative_middle.push_back(relative_path);
    }

    // Add the last point with the same yaw as the previous one
    if (!relative_middle.empty()) {
        float x = this->refPoses.back().x;
        float y = this->refPoses.back().y;

        float relative_x = this->window_width - x;
        float relative_y = (this->window_height - y) * 10;
        float last_yaw = relative_middle.back().yaw;
        relative_middle.push_back({relative_x, relative_y, last_yaw});
    }

    return relative_middle;
}

float Control::deg2rad(float angle) {
    return angle * M_PI / 180.0;
}

float Control::rad2deg(float angle) {
    return angle * 180.0 / M_PI;
}