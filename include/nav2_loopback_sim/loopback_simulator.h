// loopback_simulator.h
#ifndef NAV2_LOOPBACK_SIM_LOOPBACK_SIMULATOR_H
#define NAV2_LOOPBACK_SIM_LOOPBACK_SIMULATOR_H

#pragma once

#include <iostream>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/convert.h>
#include "tf2_ros/buffer.h"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

class LoopbackSimulator : public rclcpp::Node {
public:
    LoopbackSimulator();

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void timerCallback();

    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_; // init odom pose wrt map frame
    geometry_msgs::msg::PoseWithCovarianceStamped base_updated_pose_; // init pose of base wrt odom that keeps getting updated
    bool init_pose_set_ = false;
    bool init_odom_base_published_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_subscriber_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // NAV2_LOOPBACK_SIM_LOOPBACK_SIMULATOR_H