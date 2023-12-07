// robot_simulator.cpp
#include "nav2_loopback_sim/loopback_simulator.h"

LoopbackSimulator::LoopbackSimulator() : 
    Node("loopback_simulator_node"){

    std::cout << "Loopback simulator instance created\n";

    // Initialize ROS subscribers, publishers, and other setup
    vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LoopbackSimulator::twistCallback, this, std::placeholders::_1));

    // init pose subscriber
    init_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&LoopbackSimulator::initposeCallback, this, std::placeholders::_1));

    double timer_frequency{0.0};
    declare_parameter("timer_frequency", 10.0);  // Default frequency is 10.0 Hz
    get_parameter("timer_frequency", timer_frequency);

    // Create a timer with a callback that runs every 1 second
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / timer_frequency), std::bind(&LoopbackSimulator::timerCallback, this));

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    //tf buffer
    tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
}

void LoopbackSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    if(init_pose_set_ == false) return;
    
    double dt = 0.1; // Adjust the time step as needed

    // Transform base_updated_pose_ from "map" to "odom" frame
    geometry_msgs::msg::PoseWithCovarianceStamped odom_updated_pose_;

    geometry_msgs::msg::Pose A;
    geometry_msgs::msg::Pose B;
    try
    {
        //tf2::TimePoint transform_time = tf2::TimePointZero;

        geometry_msgs::msg::TransformStamped a_transform = tf_buffer_->lookupTransform("odom",
                              base_updated_pose_.header.frame_id,
                              tf2::TimePointZero);

        tf2::doTransform(A, B, a_transform);
        //tf2::doTransform<geometry_msgs::msg::Pose>(A, B, a_transform);
        //f_buffer_->transform(A, B, "odom");
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Could not transform point.");
    }

    odom_updated_pose_.header.stamp = this->now();  // Update the timestamp to reflect the current time

    odom_updated_pose_.pose.pose.position.x += msg->linear.x * dt;
    odom_updated_pose_.pose.pose.position.y += msg->linear.y * dt;
    odom_updated_pose_.pose.pose.position.z += msg->linear.z * dt;

    // Update orientation based on angular velocity (for simplicity, consider 2D)
    tf2::Quaternion base_link_orientation(
        odom_updated_pose_.pose.pose.orientation.x,
        odom_updated_pose_.pose.pose.orientation.y,
        odom_updated_pose_.pose.pose.orientation.z,
        odom_updated_pose_.pose.pose.orientation.w
    );

    tf2::Quaternion angular_change;
    angular_change.setRPY(msg->angular.x * dt, msg->angular.y * dt, msg->angular.z * dt);
    base_link_orientation *= angular_change;

    // Update the transformed pose in the local variable
    odom_updated_pose_.pose.pose.orientation.x = base_link_orientation.x();
    odom_updated_pose_.pose.pose.orientation.y = base_link_orientation.y();
    odom_updated_pose_.pose.pose.orientation.z = base_link_orientation.z();
    odom_updated_pose_.pose.pose.orientation.w = base_link_orientation.w();

    // Broadcast "odom" to "base_link" transform using the local variable
    geometry_msgs::msg::TransformStamped odom_to_base_transform;
    odom_to_base_transform.header.stamp = odom_updated_pose_.header.stamp;  // Use the timestamp from odom_updated_pose_
    odom_to_base_transform.header.frame_id = "odom";  // "odom" is the frame_id for the "odom" frame
    odom_to_base_transform.child_frame_id = "base_link";
    odom_to_base_transform.transform.translation.x = odom_updated_pose_.pose.pose.position.x;
    odom_to_base_transform.transform.translation.y = odom_updated_pose_.pose.pose.position.y;
    odom_to_base_transform.transform.translation.z = odom_updated_pose_.pose.pose.position.z;
    odom_to_base_transform.transform.rotation = odom_updated_pose_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_to_base_transform);
}

void LoopbackSimulator::initposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "Received initial pose wrt map");
    init_pose_ = *msg;
    base_updated_pose_ = init_pose_;
    init_pose_set_ = true;

    if(init_odom_base_published_ == false){
        // Publish odom to base_footprint transform
        geometry_msgs::msg::TransformStamped odom_to_base_transform;
        odom_to_base_transform.header.stamp = this->now();
        odom_to_base_transform.header.frame_id = "odom";
        odom_to_base_transform.child_frame_id = "base_footprint";
        odom_to_base_transform.transform.translation.x = 0.0;
        odom_to_base_transform.transform.translation.y = 0.0;
        odom_to_base_transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        odom_to_base_transform.transform.rotation.x = q.x();
        odom_to_base_transform.transform.rotation.y = q.y();
        odom_to_base_transform.transform.rotation.z = q.z();
        odom_to_base_transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_to_base_transform);
        init_odom_base_published_ = true;
    }
}

void LoopbackSimulator::timerCallback() {
    
    if(init_pose_set_ == false) return;
    
    RCLCPP_INFO(this->get_logger(), "Publishing map->odom tf");

    // Publish map to odom transform
    geometry_msgs::msg::TransformStamped map_to_odom_transform;
    map_to_odom_transform.header.stamp = this->now();
    map_to_odom_transform.header.frame_id = "map";
    map_to_odom_transform.child_frame_id = "odom";
    map_to_odom_transform.transform.translation.x = init_pose_.pose.pose.position.x;
    map_to_odom_transform.transform.translation.y = init_pose_.pose.pose.position.y;
    map_to_odom_transform.transform.translation.z = init_pose_.pose.pose.position.z;
    map_to_odom_transform.transform.rotation = init_pose_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(map_to_odom_transform);
}