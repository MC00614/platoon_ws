#pragma once

#include "utils/msg_structs.h"
#include "utils/car_struct.h"

#include "controller/stanley.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <cmath>

class Control : public rclcpp::Node {
public:
    Control();

private:
    Stanley controller;

    std::vector<Path> refPoses;

    float steerCommand;
    float current_velocity;

    bool pathValid;
    bool velocityValid;

    // Subscribe
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_subscription_;

    // Publish
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr velocity_msg);
    
    void publisher_timer_callback();
    void publish_steer(float steer);
    float quat_to_yaw(const geometry_msgs::msg::Quaternion quat);

    float normalize_steer_command(float max_steer_deg);
};