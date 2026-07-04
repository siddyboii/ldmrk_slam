#include "ldmrk_slam/isam2_optimizer.hpp"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <iostream>

using namespace gtsam;

int main() {
    std::cout << "===========================================\n";
    std::cout << " Starting ISAM2 Optimizer Headless Test \n";
    std::cout << "===========================================\n\n";

    // Initialize our custom GTSAM wrapper
    landmark_slam::Isam2Optimizer optimizer;

    // Run an initial update to process the PriorFactor on X0
    optimizer.update();
    
    std::cout << "\n--- 1. Move Forward 1 meter ---" << std::endl;
    Pose3 odom_step_1(Rot3::Identity(), Point3(1.0, 0.0, 0.0));
    optimizer.addOdometry(odom_step_1);
    optimizer.update();

    std::cout << "\n--- 2. Observe AprilTag (ID: 42) ---" << std::endl;
    // Let's say the tag is exactly 1 meter to the left of the robot's current pose
    Pose3 relative_tag_pose_1(Rot3::Identity(), Point3(0.0, 1.0, 0.0));
    optimizer.addLandmarkObservation(42, relative_tag_pose_1);
    optimizer.update();

    std::cout << "\n--- 3. Move Forward 1 meter ---" << std::endl;
    Pose3 odom_step_2(Rot3::Identity(), Point3(1.0, 0.0, 0.0));
    optimizer.addOdometry(odom_step_2);
    optimizer.update();

    std::cout << "\n--- 4. LOOP CLOSURE: Observe AprilTag (ID: 42) Again ---" << std::endl;
    // The robot has moved 1m forward. 
    // The tag was at global (1, 1). The robot is now roughly at global (2, 0).
    // So the relative pose to the tag should be 1m backward (X=-1) and 1m left (Y=1).
    Pose3 relative_tag_pose_2(Rot3::Identity(), Point3(-1.0, 1.0, 0.0));
    
    // We add some "fake" noise to our observation to force ISAM2 to do some heavy math
    // and snap the graph into alignment.
    Pose3 noisy_relative_pose(Rot3::Identity(), Point3(-1.05, 0.95, 0.0));
    optimizer.addLandmarkObservation(42, noisy_relative_pose);
    
    // This update() call will trigger the Bayes Tree loop closure optimization!
    optimizer.update();

    std::cout << "\n===========================================\n";
    std::cout << " Final Optimized Graph State \n";
    std::cout << "===========================================\n";
    
    // Extract and print the final trajectory and map
    Values final_state = optimizer.getOptimizedState();
    final_state.print();

    return 0;
}