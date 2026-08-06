#include "ldmrk_slam/slam_node.hpp"

namespace landmark_slam {

LandmarkSlamNode::LandmarkSlamNode() : Node("landmark_slam_node"), first_odom_received_(false) {
    // 1. Initialize our ISAM2 Engine
    optimizer_ = std::make_unique<Isam2Optimizer>();

    // 1.5 Initialize TF2 Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // NOTE: Change this if your camera link has a different name in your URDF!
    camera_frame_id_ = "base_footprint"; 

    // 2. Setup Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LandmarkSlamNode::odomCallback, this, std::placeholders::_1));

    apriltag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10, std::bind(&LandmarkSlamNode::aprilTagCallback, this, std::placeholders::_1));

    // 3. Setup Publishers for RViz
    optimized_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/optimized_path", 10);
    landmarks_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/optimized_landmarks", 10);

    // 4. Setup Optimization Loop (Run at 10 Hz)
    // We don't want to trigger an ISAM2 update on every single odometry message (too fast).
    // A 10Hz timer provides a very smooth, real-time RViz experience.
    optimization_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LandmarkSlamNode::optimizationTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Landmark SLAM Node Started!");
}

void LandmarkSlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    gtsam::Pose3 current_odom_pose = rosPoseToGtsam(msg->pose.pose);

    if (!first_odom_received_) {
        last_odom_pose_ = current_odom_pose;
        first_odom_received_ = true;
        return;
    }

    // CRITICAL: Calculate the relative movement since the last callback
    // mathematically this is: Delta = Last^-1 * Current
    gtsam::Pose3 relative_odom_step = last_odom_pose_.between(current_odom_pose);

    // Only add to the graph if the robot actually moved
    if (relative_odom_step.translation().norm() > 0.01 || 
        relative_odom_step.rotation().xyz().norm() > 0.01) {
        
        optimizer_->addOdometry(relative_odom_step);
        last_odom_pose_ = current_odom_pose;
    }
}

/*void LandmarkSlamNode::aprilTagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    for (const auto& detection : msg->detections) {
        int tag_id = detection.id;
        
        // The pose of the tag relative to the camera
        gtsam::Pose3 relative_tag_pose = rosPoseToGtsam(detection.pose.pose.pose);
        
        optimizer_->addLandmarkObservation(tag_id, relative_tag_pose);
        RCLCPP_INFO(this->get_logger(), "Sent Tag %d to Optimizer", tag_id);
    }
}*/

void LandmarkSlamNode::aprilTagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    for (const auto& detection : msg->detections) {
        int tag_id = detection.id;
        std::string family = detection.family;
        
        // Standard apriltag_ros frame naming convention (e.g., "tag36h11:42")
        std::string tag_frame = family + ":" + std::to_string(tag_id);
        
        try {
            // Lookup the 3D transform from the camera to the tag
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                camera_frame_id_, tag_frame, tf2::TimePointZero);
            
            // Convert the ROS Transform to a GTSAM Pose3
            gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
                t.transform.rotation.w, t.transform.rotation.x, 
                t.transform.rotation.y, t.transform.rotation.z);
                
            gtsam::Point3 trans(
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            
            gtsam::Pose3 relative_tag_pose(rot, trans);
            
            optimizer_->addLandmarkObservation(tag_id, relative_tag_pose);
            RCLCPP_INFO(this->get_logger(), "Sent Tag %d to Optimizer", tag_id);
            
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not find TF for tag %d: %s", tag_id, ex.what());
        }
    }
}

void LandmarkSlamNode::optimizationTimerCallback() {
    // 1. Trigger the math engine
    optimizer_->update();

    // 2. Retrieve the results
    gtsam::Values optimized_state = optimizer_->getOptimizedState();

    // 3. Prepare ROS Messages for RViz
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom"; // Anchor it to the global frame

    visualization_msgs::msg::MarkerArray markers_msg;

    // 4. Iterate through everything GTSAM has estimated
    for (const auto& key_value : optimized_state) {
        gtsam::Symbol symbol(key_value.key);
        
        // Extract Poses ('x') for the Trajectory Path
        if (symbol.chr() == 'x') {
            gtsam::Pose3 pose = key_value.value.cast<gtsam::Pose3>();
            
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose = gtsamPoseToRos(pose);
            path_msg.poses.push_back(pose_stamped);
        }
        
        // Extract Landmarks ('l') for the AprilTag cubes
        else if (symbol.chr() == 'l') {
            gtsam::Pose3 tag_pose = key_value.value.cast<gtsam::Pose3>();
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "apriltags";
            marker.id = symbol.index();
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = gtsamPoseToRos(tag_pose);
            marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.05; // 20cm tag
            marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green
            
            markers_msg.markers.push_back(marker);
        }
    }

    // 5. Publish!
    if (!path_msg.poses.empty()) {
        optimized_path_pub_->publish(path_msg);
    }
    if (!markers_msg.markers.empty()) {
        landmarks_pub_->publish(markers_msg);
    }
}

// --- Converters ---
gtsam::Pose3 LandmarkSlamNode::rosPoseToGtsam(const geometry_msgs::msg::Pose& ros_pose) {
    return gtsam::Pose3(
        gtsam::Rot3::Quaternion(
            ros_pose.orientation.w, 
            ros_pose.orientation.x, 
            ros_pose.orientation.y, 
            ros_pose.orientation.z),
        gtsam::Point3(
            ros_pose.position.x, 
            ros_pose.position.y, 
            ros_pose.position.z)
    );
}

geometry_msgs::msg::Pose LandmarkSlamNode::gtsamPoseToRos(const gtsam::Pose3& gtsam_pose) {
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = gtsam_pose.x();
    ros_pose.position.y = gtsam_pose.y();
    ros_pose.position.z = gtsam_pose.z();
    
    auto quat = gtsam_pose.rotation().toQuaternion();
    ros_pose.orientation.w = quat.w();
    ros_pose.orientation.x = quat.x();
    ros_pose.orientation.y = quat.y();
    ros_pose.orientation.z = quat.z();
    
    return ros_pose;
}

} // namespace landmark_slam

// --- Main Execution ---
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<landmark_slam::LandmarkSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}