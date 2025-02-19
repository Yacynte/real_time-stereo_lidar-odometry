cmake_minimum_required(VERSION 3.10)
project(StereoLiDARRecorder)

# Check if cross-compiling
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Cross-compiling for Raspberry Pi 4")
else()
    message(STATUS "Building for x86_64 PC")
endif()

# Ensure C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io people)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(StereoLiDARRecorder src/main.cpp src/StereoCamera.cpp src/LiDAR.cpp)

# Link libraries
target_link_libraries(StereoLiDARRecorder ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
