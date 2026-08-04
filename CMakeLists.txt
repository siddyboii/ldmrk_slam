cmake_minimum_required(VERSION 3.10)
project(ldmrk_slam)

# Set C++ Standard
# =====================================================
# C++ Standard
# =====================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =====================================================
# Compiler Warnings
# =====================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================
# Find Packages
# =====================================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 1. Find GTSAM (Must be installed on the system)
find_package(GTSAM REQUIRED)

# 3. Create the Core Engine Library
add_library(optimizer_core src/isam2_optimizer.cpp)

# 2. Include Directories
# include_directories(include)
target_include_directories(optimizer_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


target_link_libraries(optimizer_core PUBLIC gtsam)

# =====================================================
# Main SLAM Node
# =====================================================
add_executable(slam_node
    src/slam_node.cpp
    src/main.cpp
)

target_link_libraries(slam_node
        optimizer_core
)

ament_target_dependencies(slam_node
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    apriltag_msgs
    tf2_ros
)
    
# 4. Create the Test Executable
# =====================================================
# Unit Test Executable
# =====================================================
add_executable(test_optimizer tests/test_optimizer.cpp)
target_link_libraries(test_optimizer PUBLIC optimizer_core)

# =====================================================
# Install
# =====================================================
install(
    TARGETS
        optimizer_core
        slam_node
        test_optimizer
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

# =====================================================
# Export
# =====================================================
ament_export_include_directories(include)

ament_package()