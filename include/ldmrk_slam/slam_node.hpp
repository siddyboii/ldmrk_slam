#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
// TF2 Includes for 3D Pose Lookup
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h> // ADDED FOR MAP->ODOM

// Our custom GTSAM engine
#include "ldmrk_slam/isam2_optimizer.hpp"
namespace landmark_slam  {

class LandmarkSlamNode : public rclcpp::Node {
public:
    LandmarkSlamNode();

private:
    // --- Callbacks ---
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void aprilTagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void optimizationTimerCallback();

    // --- Helper Functions ---
    gtsam::Pose3 rosPoseToGtsam(const geometry_msgs::msg::Pose& ros_pose);
    geometry_msgs::msg::Pose gtsamPoseToRos(const gtsam::Pose3& gtsam_pose);

    // --- TF2 Interfaces ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // ADDED
    std::string camera_frame_id_;

    // --- ROS 2 Interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_; // ADDED
    
    rclcpp::TimerBase::SharedPtr optimization_timer_;

    // --- Core Engine & State ---
    std::unique_ptr<Isam2Optimizer> optimizer_;
    
    bool first_odom_received_;
    gtsam::Pose3 last_odom_pose_;
};

} // namespace landmark_slam 