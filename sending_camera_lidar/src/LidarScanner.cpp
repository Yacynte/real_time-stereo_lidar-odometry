// LiDAR.cpp
#include "LidarScanner.h"


using namespace rp::standalone::rplidar;



LidarScanner::LidarScanner(const std::string& port)
    : port_(port), baudrate_(460800), lidar_(nullptr) {}

LidarScanner::~LidarScanner() {
    if (lidar_) {
        lidar_->stopMotor();
        lidar_->stop();
        lidar_->disconnect();
        RPlidarDriver::DisposeDriver(lidar_);
        lidar_ = nullptr;
    }
}


bool LidarScanner::initialize() {
    lidar_ = RPlidarDriver::CreateDriver();
    if (!lidar_) {
        std::cerr << "Failed to create RPLidar driver instance." << std::endl;
        return false;
    }

    if (IS_FAIL(lidar_->connect(port_.c_str(), baudrate_))) {
        std::cerr << "Failed to connect to RPLidar on " << port_ << std::endl;
        RPlidarDriver::DisposeDriver(lidar_);
        lidar_ = nullptr;
        return false;
    }
    // std::cout << "Attempting to start motor!" << std::endl;
    lidar_->startMotor();
    // std::cout << "Motor started successfully!" << std::endl;
    // std::cout << "Attempting to start scan!" << std::endl;
    lidar_->startScan(false, true);
    // std::cout << "Scan started successfully!" << std::endl;

    return true;
}

bool LidarScanner::getScans(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_) {
    if (!lidar_) return false;

    

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    if (IS_FAIL(lidar_->grabScanDataHq(nodes, count))) {
        std::cerr << "Failed to grab scan data." << std::endl;
        lidar_->stop();
        lidar_->stopMotor();
        return false;
    }

    // Ensure cloud_ is valid
    if (!cloud_) {
        cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    cloud_->clear();

    for (size_t i = 0; i < count; i++) {
        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float distance = nodes[i].dist_mm_q2 / 4.0f;

        if (distance > 0) {
            float x = distance * cos(angle * M_PI / 180.0f);
            float y = distance * sin(angle * M_PI / 180.0f);
            cloud_->points.push_back(pcl::PointXYZ(x, y, 0.0));
        }
    }

    if(!cloud_ || cloud_->empty()) return false;
    // std::cout << "Cloud size using .size(): " << cloud_->points.size() << std::endl;
    return true;
}
