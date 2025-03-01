cmake_minimum_required(VERSION 3.10)
project(receiving_camera_lidar)
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
add_definitions(${PCL_DEFINITIONS})


# Add executable
add_executable(receiving_camera_lidar src/main.cpp)

# Link libraries
target_link_libraries(receiving_camera_lidar 
                    ${OpenCV_LIBS}          # Link the opencv SDK static library
                    ${PCL_LIBRARIES}        # Link the pcl SDK static library
                    pthread
                    )

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS}              # Include opencv headers
                    ${PCL_INCLUDE_DIRS}                 # Include PCL headers
                    # ${CMAKE_SOURCE_DIR}/include         # Include MY headers
                    )
