#include "control.h"

Control::Control() : rclcpp::Node("vehicle_control") {
    double k = 1.5;
    double ks = 5.2;

    this->controller = Stanley(k, ks);

    this->pathValid = false;
    this->velocityValid = false;

    // Subscribe
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/truck0/path", 10, std::bind(&Control::path_callback, this,  std::placeholders::_1));
    velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "/truck0/velocity", 10, std::bind(&Control::velocity_callback, this, std::placeholders::_1));

    // Publish
    steer_publisher_ = this->create_publisher<example_interfaces::msg::Float64>(
         "/truck0/steer_control", 10);

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
        refPose.y = path_msg->poses[i].pose.position.y;
        refPose.yaw = this->quat_to_yaw(path_msg->poses[i].pose.orientation);
        std::cout << "X : " << refPose.x << " Y : " << refPose.y << " Yaw : " << refPose.yaw << std::endl;
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
    if (!this->pathValid) { std::cout << "Message Receive Error" << std::endl; return;}

    this->controller.stanley_control(this->refPoses, this->current_velocity);

    this->steerCommand = this->controller.getDelta();

    this->publish_steer(this->steerCommand);
}

void Control::publish_steer(float steer) {
    auto steer_msg = std::make_unique<example_interfaces::msg::Float64>();
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
