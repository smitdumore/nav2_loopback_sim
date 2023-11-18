// robot_simulator.cpp
#include "nav2_loopback_sim/loopback_simulator.h"

LoopbackSimulator::LoopbackSimulator(const geometry_msgs::msg::PoseWithCovarianceStamped &odom_pose_msg) : 
    Node("loopback_simulator_node"), odom_pose_{odom_pose_msg}{

    std::cout << "Loopback simulator instance created\n";

    // Initialize ROS subscribers, publishers, and other setup
    vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LoopbackSimulator::twistCallback, this, std::placeholders::_1));

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // Initialize init pose publisher
    init_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    
    std::cout << "sleeping for 2 secs\n"; 
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Publish the initial TF between "map" and "odom"
    publishInitial_TF_pose();
}

void LoopbackSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    //update "odom" -> "base_link" tf

    std::cout << msg->linear.x;
}

void LoopbackSimulator::publishInitial_TF_pose() {

    
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = odom_pose_.pose.pose.position.x;
        transformStamped.transform.translation.y = odom_pose_.pose.pose.position.y;
        transformStamped.transform.translation.z = odom_pose_.pose.pose.position.z;
        transformStamped.transform.rotation = odom_pose_.pose.pose.orientation;        

        std::cout << "published init pose and tf\n";
        init_pose_publisher_->publish(odom_pose_);


        std::this_thread::sleep_for(std::chrono::seconds(5));


        tf_broadcaster_->sendTransform(transformStamped);

    
}