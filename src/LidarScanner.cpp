// LiDAR.cpp
#include "LidarScanner.h"


using namespace rp::standalone::rplidar;

// Constructor
/*
LidarScanner1::LidarScanner1(const std::string &port) : port_name(port), drv(nullptr), isConnected(false) {
    std::cout << "LiDAR initialized on port: " << port_name << std::endl;
}
// Constructor definition
// LidarScanner::LidarScanner(const std::string& port_name) : port(port_name) {
//     std::cout << "LiDAR initialized on port: " << port << std::endl;
// }
// Destructor
LidarScanner1::~LidarScanner1() {
    if (drv) {
        // drv->stop();
        // drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
    }
}

// Initialize LIDAR
bool LidarScanner1::initialize() {
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        std::cerr << "Failed to create RPLIDAR driver." << std::endl;
        return false;
    }

    if (IS_FAIL(drv->connect(port_name.c_str(), 115200))) {
        std::cerr << "Failed to connect to LIDAR on " << port_name << std::endl;
        return false;
    }

    // Start scanning
    // drv->startMotor();
    drv->startScan(0, 1);
    isConnected = true;
    return true;
}

// Obtain LIDAR Scans
bool LidarScanner1::getScans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!isConnected) return false;

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    if (IS_OK(drv->grabScanDataHq(nodes, count))) {
        drv->ascendScanData(nodes, count);
        scans.clear(); // Clear previous scans
        for (size_t i = 0; i < count; i++) {
            float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
            float distance = nodes[i].dist_mm_q2 / 4.0f;
            scans.push_back({angle, distance});
        }
        cloud = LidarScanner1::convertScanToPointCloud();
        return true;
    }
    return false;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr LidarScanner1::convertScanToPointCloud() {
    // Create a new point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& scan : scans) {
        if (scan.distance <= 0) continue;  // Ignore invalid readings

        pcl::PointXYZ point;
        
        // Convert polar coordinates to Cartesian
        float angle_rad = scan.angle * M_PI / 180.0f;  // Convert degrees to radians
        point.x = scan.distance * std::cos(angle_rad);
        point.y = scan.distance * std::sin(angle_rad);
        point.z = 0.0f;  // Assuming 2D lidar (no height info)

        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

*/
// Define a function to downsample the point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_odometry::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filteredCloud);
    return filteredCloud;
}

// Define a function to extract edge and surface features (placeholder)
void lidar_odometry::extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &edges, 
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &planes) {
    // Placeholder: In real LOAM, you apply curvature analysis to separate sharp edges and flat planes
    *edges = *cloud;  
    *planes = *cloud; 
}

// Function to estimate motion using ICP
Eigen::Matrix4f lidar_odometry::estimateMotion(pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
                               pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-6);
    icp.setMaximumIterations(50);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    return icp.getFinalTransformation();
}

std::pair<cv::Mat, cv::Mat> lidar_odometry::lidar_odom(pcl::PointCloud<pcl::PointXYZ>::Ptr &scan1_org, 
                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &scan2_org, cv::Mat init_R, cv::Mat init_T ){
    // Load two consecutive point clouds (Replace with real LIDAR scans)
    std::cout << "Creating emty copies...."<<std::endl;
    // Create local copies
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan2(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << " copies created!!"<<std::endl;
    std::cout << " downsampling original 1 to copy !!"<<std::endl;
    // Downsample the copies (originals remain unchanged)
    scan1 = downsampleCloud(scan1_org);
    std::cout << " downsampling original 2 to copy !!"<<std::endl;
    scan2 = downsampleCloud(scan2_org);
    std::cout << " downsampling ocomplete !!"<<std::endl;

    // Downsample the point clouds
    // scan1 = downsampleCloud(scan1);
    // scan2 = downsampleCloud(scan2);

    // Extract features (sharp edges & planes)
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planes1(new pcl::PointCloud<pcl::PointXYZ>);
    extractFeatures(scan1, edges1, planes1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr edges2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planes2(new pcl::PointCloud<pcl::PointXYZ>);
    extractFeatures(scan2, edges2, planes2);

    // Estimate motion using ICP on edge features
    transformation = estimateMotion(edges1, edges2);

    // Convert Eigen to OpenCV
    rotation_matrix = cv::Mat(3, 3, CV_32F);
    translation_vector = cv::Mat(3, 1, CV_32F);

    // Copy data
    memcpy(init_R.data, transformation.data(), sizeof(float) * 9);
    memcpy(init_T.data, transformation.data(), sizeof(float) * 3);

    translation_vector += init_T;
    rotation_matrix = init_R*rotation_matrix;
    return std::make_pair(translation_vector, rotation_matrix);
    // Print the estimated transformation (R, T)
    // cout << "Estimated Transformation:\n" << transformation << endl;

    // return 0;
}



LidarScanner::LidarScanner(const std::string& port)
    : port_(port), baudrate_(460800), lidar_(nullptr) {}

LidarScanner::~LidarScanner() {
    if (lidar_) {
        lidar_->disconnect();
        RPlidarDriver::DisposeDriver(lidar_);
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

    return true;
}

bool LidarScanner::getScans(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_) {
    if (!lidar_) return false;

    std::cout << "Attempting to start motor!" << std::endl;
    lidar_->startMotor();
    std::cout << "Motor started successfully!" << std::endl;
    std::cout << "Attempting to start scan!" << std::endl;
    lidar_->startScan(false, true);
    std::cout << "Scan started successfully!" << std::endl;

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

    std::cout << "Cloud size using .size(): " << cloud_->points.size() << std::endl;
    return true;
}
