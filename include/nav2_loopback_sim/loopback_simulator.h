// loopback_simulator.h
#pragma once

#include <iostream>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LoopbackSimulator : public rclcpp::Node {
public:
    LoopbackSimulator();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void timerCallback();

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_; // init odom pose wrt map frame
    bool init_pose_set_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_subscriber_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};
