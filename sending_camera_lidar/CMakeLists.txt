cmake_minimum_required(VERSION 3.10)
project(sending_camera_lidar)
# Set the build type to Debug
set(CMAKE_BUILD_TYPE Debug)
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
find_package(PCL REQUIRED COMPONENTS common io people filters)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
link_directories(${RPLIDAR_SDK_LIB_DIR})        # Add the RPLIDAR SDK library directory to the linker search path
add_definitions(${PCL_DEFINITIONS})


# Add executable
add_executable(sending_camera_lidar src/main.cpp src/StereoCamera.cpp src/LidarScanner.cpp)

# Link libraries
target_link_libraries(sending_camera_lidar 
                    ${OpenCV_LIBS}          # Link the opencv SDK static library
                    ${PCL_LIBRARIES}        # Link the pcl SDK static library
                    pthread
                    ${RPLIDAR_SDK_LIB_DIR}/libsl_lidar_sdk.so     # Link the RPLIDAR SDK dynamic library
                    )

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include         # Include MY headers
                    ${OpenCV_INCLUDE_DIRS}              # Include opencv headers
                    ${PCL_INCLUDE_DIRS}                 # Include PCL headers
                    ${RPLIDAR_SDK_DIR}/sdk/include      # Include RPLIDAR SDK headers
                    ${RPLIDAR_SDK_DIR}/sdk/src          # Include RPLIDAR SDK headers
                    )
