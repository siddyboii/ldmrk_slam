cmake_minimum_required(VERSION 3.10)
project(tricycle_slam_core_test)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find GTSAM (Must be installed on the system)
find_package(GTSAM REQUIRED)

# 2. Include Directories
include_directories(include)

# 3. Create the Core Engine Library
add_library(optimizer_core src/isam2_optimizer.cpp)
target_link_libraries(optimizer_core PUBLIC gtsam)
    
# 4. Create the Test Executable
add_executable(test_optimizer tests/test_optimizer.cpp)
target_link_libraries(test_optimizer PUBLIC optimizer_core)