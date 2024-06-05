#include "control.h"

Control::Control(int truck_id) : rclcpp::Node("truck" + std::to_string(truck_id) + "_lateral_control"), truck_id(truck_id) {
    double k = 1.5;
    double ks = 10.2;

    this->controller = Stanley(k, ks);

    this->pathValid = false;
    this->velocityValid = false;

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
        std::chrono::milliseconds(100),
        std::bind(&Control::publisher_timer_callback, this)
    );
}

void Control::path_callback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    this->pathValid = false;
    this->refPoses.clear();
    Path refPose;
    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
        refPose.x = path_msg->poses[i].pose.position.x;
        refPose.y = path_msg->poses[i].pose.position.y * 10.0;
        refPose.yaw = this->quat_to_yaw(path_msg->poses[i].pose.orientation);
        // std::cout << "X : " << refPose.x << " Y : " << refPose.y << " Yaw : " << refPose.yaw << std::endl;
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

    this->controller.stanley_control(this->refPoses, this->current_velocity);

    this->steerCommand = this->controller.getDelta();
    float normalize_steer = normalize_steer_command(3.5);

    // std::cout << "Steer Command : " << this->steerCommand * 180.0 / M_PI <<std::endl;
    // std::cout << "Decision Steer : " << normalize_steer << std::endl;

    this->publish_steer(normalize_steer);
}

void Control::publish_steer(float steer) {
    auto steer_msg = std::make_unique<std_msgs::msg::Float32>();
    steer_msg->data = steer;
    std::cout << " Publish Steering : " << steer << std::endl;
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
    // float max_steer_rad = max_steer_deg * M_PI /180.0;

    // Normalize the steer to the range -30 to 30

    float steer_deg = ((this->steerCommand * 180.0 / M_PI) * (-1) + 90.0);

    if (steer_deg > max_steer_deg) {steer_deg = max_steer_deg;}
    else if (steer_deg < max_steer_deg * (-1)) {steer_deg = max_steer_deg * (-1);}
    std::cout << "Steer Command : " << steer_deg <<std::endl;

    steer_deg *= 8.0;

    return steer_deg;
}