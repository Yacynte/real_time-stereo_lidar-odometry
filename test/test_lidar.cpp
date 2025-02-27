#include "rplidar.h"
#include <iostream>

int main() {
    rp::standalone::rplidar::RPlidarDriver *driver = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
    if (!driver) {
        std::cerr << "Failed to create RPLIDAR driver." << std::endl;
        return -1;
    }
    std::cout << "RPLIDAR driver created successfully." << std::endl;
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(driver);
    return 0;
}

/*
g++ -o test_lidar test_lidar.cpp \
    -I/home/divan/stereo_lidar_recorder/include/rplidar_sdk/sdk/include \
    -I/home/divan/stereo_lidar_recorder/include/rplidar_sdk/sdk/src \
    -I/usr/include/pcl-1.10 \
    -I/usr/include/eigen3 \
    -L/home/divan/stereo_lidar_recorder/include/rplidar_sdk/obj/Linux/Release \
     -L/home/divan/stereo_lidar_recorder/include/rplidar_sdk/output/Linux/Release/sdk/src \
    -L/usr/lib/x86_64-linux-gnu \
    -lsl_lidar_sdk -sl_lidar_driver -lpcl_common -lpcl_io -lpthread
*/