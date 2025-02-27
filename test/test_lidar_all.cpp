#include <iostream>
#include <sys/stat.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "rplidar.h" // Include RPLIDAR SDK header

using namespace rp::standalone::rplidar;

bool fileExists(const char* filename) {
    struct stat buffer;
    return (stat(filename, &buffer) == 0);
}

int main() {
    // Create a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Initialize RPLIDAR driver
    RPlidarDriver *driver = RPlidarDriver::CreateDriver();
    if (!driver) {
        std::cerr << "Failed to create RPLIDAR driver." << std::endl;
        return -1;
    }
    std::cout << "RPLIDAR driver created successfully." << std::endl;
    std::cout << "Attempting to connect to RPLIDAR..." << std::endl;

    
    // if (!fileExists("/dev/ttyUSB0")) {
    //     std::cerr << "Error: /dev/ttyUSB0 does not exist! Is the LiDAR connected?" << std::endl;
    //     RPlidarDriver::DisposeDriver(driver);
    //     return -1;
    // }



    // Connect to the RPLIDAR
    if (IS_FAIL(driver->connect("/dev/ttyUSB0", 460800))) {
        std::cerr << "Failed to connect to RPLIDAR." << std::endl;
        RPlidarDriver::DisposeDriver(driver);
        return -1;
    }


    std::cout << "Connected! Checking health status..." << std::endl;

    // Check LIDAR health status
    rplidar_response_device_health_t healthinfo;
    if (IS_FAIL(driver->getHealth(healthinfo))) {
        std::cerr << "Error: Failed to retrieve device health." << std::endl;
        RPlidarDriver::DisposeDriver(driver);
        return -1;
    }
    if (healthinfo.status != RPLIDAR_STATUS_OK) {
        std::cerr << "Warning: RPLIDAR health check reported an issue!" << std::endl;
    }


    std::cout << "Attenpting to start motor!" << std::endl;
    // Start scanning
    driver->startMotor();
    std::cout << "start Motor successfully!" << std::endl;
    std::cout << "Attenpting to start Scan!" << std::endl;
    driver->startScan(false, true); // Add arguments: force = false, useTypicalScan = true
    std::cout << "start Scan successfully!" << std::endl;

    // Grab scan data
    rplidar_response_measurement_node_hq_t nodes[8192];
    // size_t count = _countof(nodes);
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    if (IS_FAIL(driver->grabScanDataHq(nodes, count))) {
        std::cerr << "Failed to grab scan data." << std::endl;
        driver->stop();
        driver->stopMotor();
        RPlidarDriver::DisposeDriver(driver);
        return -1;
    }

    // Convert scan data to PCL cloud
    for (size_t i = 0; i < count; ++i) {
        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float distance = nodes[i].dist_mm_q2 / 4.0f;
        pcl::PointXYZ point;
        point.x = distance * cos(angle * M_PI / 180.0f);
        point.y = distance * sin(angle * M_PI / 180.0f);
        point.z = 0; // 2D LiDAR, so z = 0
        cloud->points.push_back(point);
    }

    // Save the point cloud to a PCD file
    // pcl::io::savePCDFileASCII("lidar_scan.pcd", *cloud);
    std::cout << "Saved " << cloud->points.size() << " points to lidar_scan.pcd." << std::endl;

    // Stop scanning and clean up
    driver->stop();
    driver->stopMotor();
    RPlidarDriver::DisposeDriver(driver);

    return 0;
}