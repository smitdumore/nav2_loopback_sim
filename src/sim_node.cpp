#include "nav2_loopback_sim/loopback_simulator.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.frame_id = "map"; 
        
    // Set the position (x, y, z)
    pose_msg.pose.pose.position.x = 1.0;
    pose_msg.pose.pose.position.y = 1.0;
    pose_msg.pose.pose.position.z = 0.0;
        
    // Set the orientation (quaternion)
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

    auto node = std::make_shared<LoopbackSimulator>(pose_msg);  // Create an instance of your simulator class

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}