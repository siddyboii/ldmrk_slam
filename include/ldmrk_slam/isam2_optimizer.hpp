#pragma once

//Standard C++
#include <unordered_set>
#include <memory>

//GTSAM Core
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

//GTSAM Geometry and Factors
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>


namespace landmark_slam {

class Isam2Optimizer {
public:
    /**
     * @brief Constructor. Initializes the ISAM2 engine, sets up the noise models,
     * and anchors the very first pose (X0) to the origin with a PriorFactor.
     */
    Isam2Optimizer();

    /**
     * @brief Ingests an odometry measurement.
     * @param odom_step The RELATIVE movement since the last pose.
     */
    void addOdometry(const gtsam::Pose3& odom_step);

    /**
     * @brief Ingests an AprilTag observation.
     * @param tag_id The ID of the detected tag.
     * @param relative_pose The 3D pose of the tag relative to the camera.
     */
    void addLandmarkObservation(int tag_id, const gtsam::Pose3& relative_pose);

    /**
     * @brief Feeds the temporary graph and values into the ISAM2 engine,
     * performs the incremental optimization, and clears the temporary buffers.
     */
    void update();

    /**
     * @brief Retrieves the latest optimized trajectory and map.
     * @return gtsam::Values containing all optimized poses and landmarks.
     */
    gtsam::Values getOptimizedState() const;

private:
    // --- The Core Engine ---
    gtsam::ISAM2 isam_;

    // --- Step 1: The Accumulators (Temporary Buffers) ---
    // These hold ONLY the brand new data since the last update() call.
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_initial_estimates_;

    // --- State Tracking ---
    int current_pose_index_; 
    gtsam::Pose3 current_estimate_; // Keeps track of our rough dead-reckoning position
    
    // We need to know if a tag is brand new (requires an initial guess in Values)
    // or if we have seen it before (Loop Closure - no initial guess needed).
    std::unordered_set<int> seen_landmarks_;

    // --- Noise Models (The Spring Stiffness) ---
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr observation_noise_;

    // Helper functions for GTSAM Symbols
    // 'x' for poses, 'l' for landmarks
    inline gtsam::Symbol X(int index) const { return gtsam::Symbol('x', index); }
    inline gtsam::Symbol L(int id) const { return gtsam::Symbol('l', id); }
};

} // namespace landmark_slam