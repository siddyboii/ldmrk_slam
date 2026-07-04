#include "ldmrk_slam/isam2_optimizer.hpp"
#include <iostream>

namespace landmark_slam {

Isam2Optimizer::Isam2Optimizer() {
    // 1. Initialize ISAM2 Engine
    // We set up parameters to tell iSAM2 how aggressively to relinearize the tree
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 10;
    isam_ = gtsam::ISAM2(parameters);

    // 2. Initialize State Tracking
    current_pose_index_ = 0;
    current_estimate_ = gtsam::Pose3::Identity(); // Start exactly at [0,0,0] with no rotation

    // 3. Define Noise Models (The Spring Stiffness: Sigma matrices)
    // The vector format is [roll, pitch, yaw, x, y, z] representing standard deviations.
    // Smaller numbers = Stiffer spring (higher trust).
    
    // PRIOR NOISE: We are 99.9% confident the robot starts EXACTLY at the origin.
    gtsam::Vector6 prior_sigmas;
    prior_sigmas << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01; 
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);

    // ODOMETRY NOISE: Tricycle drives drift a lot, especially in Yaw (rotation) and X/Y.
    // We tell the graph not to trust our odometry very much.
    gtsam::Vector6 odom_sigmas;
    odom_sigmas << 0.05, 0.05, 0.2, 0.3, 0.3, 0.1; 
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);

    // OBSERVATION NOISE: AprilTags provide highly accurate relative measurements.
    gtsam::Vector6 obs_sigmas;
    obs_sigmas << 0.02, 0.02, 0.02, 0.05, 0.05, 0.05;
    observation_noise_ = gtsam::noiseModel::Diagonal::Sigmas(obs_sigmas);

    // 4. Anchor the Graph
    // If we don't pin the first node, the entire map could float infinitely through space.
    new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        X(current_pose_index_), current_estimate_, prior_noise_);
    
    // 5. Provide the initial guess for X0
    new_initial_estimates_.insert(X(current_pose_index_), current_estimate_);
    
    std::cout << "[ISAM2] Optimizer Initialized. Anchored X0." << std::endl;
}

void Isam2Optimizer::addOdometry(const gtsam::Pose3& odom_step) {
    // 1. Advance our state machine
    int previous_pose_index = current_pose_index_;
    current_pose_index_++;

    // 2. Create the Odometry Spring (BetweenFactor)
    // This mathematically links X(i-1) to X(i) with the loose odom_noise_ spring.
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(previous_pose_index), X(current_pose_index_), odom_step, odom_noise_);

    // 3. Calculate the Initial Guess for the new pose (Dead Reckoning)
    // CRITICAL: Notice we use the `*` operator, not `+`. 
    // In GTSAM, `*` for Pose3 invokes the Lie Group manifold multiplication (the Exponential map bridge we learned in Module 1.4!)
    current_estimate_ = current_estimate_ * odom_step;

    // 4. Add the new guess to the accumulator
    new_initial_estimates_.insert(X(current_pose_index_), current_estimate_);
    
    std::cout << "[ISAM2] Added Odometry. New Pose Index: " << current_pose_index_ << std::endl;
}

void Isam2Optimizer::addLandmarkObservation(int tag_id, const gtsam::Pose3& relative_pose) {
    // 1. Add the Observation Factor (The Stiff Spring)
    // Connects the current robot pose X(i) to the landmark L(tag_id)
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(current_pose_index_), L(tag_id), relative_pose, observation_noise_);

    // 2. Loop Closure Check
    if (seen_landmarks_.count(tag_id) == 0) {
        // BRAND NEW LANDMARK
        // We must calculate an initial guess for its location in the global world.
        // World_Pose_of_Tag = World_Pose_of_Robot * Robot_to_Tag_Relative_Pose
        gtsam::Pose3 estimated_landmark_pose = current_estimate_ * relative_pose;
        
        new_initial_estimates_.insert(L(tag_id), estimated_landmark_pose);
        seen_landmarks_.insert(tag_id);
        
        std::cout << "[ISAM2] Discovered NEW Landmark: " << tag_id << std::endl;
    } else {
        // LOOP CLOSURE
        // We have seen this before. We DO NOT add an initial guess.
        // The graph already knows where L(tag_id) roughly is. 
        // Adding the factor above will force ISAM2 to pull the drifted robot trajectory 
        // into alignment with this known landmark.
        std::cout << "[ISAM2] LOOP CLOSURE on Landmark: " << tag_id << std::endl;
    }
}

void Isam2Optimizer::update() {
    // 1. Feed the new data into the ISAM2 Engine
    // This triggers the surgical Bayes Tree optimization.
    // It only updates the branches of the matrix affected by the new data.
    isam_.update(new_factors_, new_initial_estimates_);

    // 2. Clear the temporary accumulators for the next step
    new_factors_.resize(0);
    new_initial_estimates_.clear();

    // 3. Update our current dead-reckoning estimate with the newly optimized pose.
    // This is vital: it ensures our next odometry step builds off the CORRECTED 
    // position, preventing odometry drift from accumulating uncontrollably.
    current_estimate_ = isam_.calculateEstimate<gtsam::Pose3>(X(current_pose_index_));
}

gtsam::Values Isam2Optimizer::getOptimizedState() const {
    // Retrieves the entire optimized trajectory and map
    return isam_.calculateEstimate();
}


} 