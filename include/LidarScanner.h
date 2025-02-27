// LiDAR.h
#pragma once
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/common/io.h>  // For copyPointCloud
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "rplidar.h"


using namespace rp::standalone::rplidar;

class LidarScanner1 {
    public:
        // Constructor
        // Constructor declaration
        explicit LidarScanner1(const std::string& port);

        // Destructor
        ~LidarScanner1();

        // Initialize LIDAR
        bool initialize();

        // Get LIDAR scan data
        bool getScans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    private:
        struct ScanData {
            float angle;
            float distance;
        };
        RPlidarDriver *drv;
        bool isConnected;
        pcl::PointCloud<pcl::PointXYZ>::Ptr convertScanToPointCloud();
        std::string port_name;  // Store port name
        std::vector<ScanData> scans;
};



class lidar_odometry {
    public:
    std::pair<cv::Mat, cv::Mat> lidar_odom(pcl::PointCloud<pcl::PointXYZ>::Ptr &scans_pre, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &scans_cur, cv::Mat init_R, cv::Mat init_T);
    private:
        // Define a function to downsample the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        // Define a function to extract edge and surface features (placeholder)
        void extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr &edges, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr &planes);
        // Function to estimate motion using ICP
        Eigen::Matrix4f estimateMotion(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target);
        Eigen::Matrix4f transformation;
        cv::Mat rotation_matrix;                // Sotre Rotation matrix 
        cv::Mat translation_vector;
        
};

class LidarScanner {
    public:
        LidarScanner(const std::string& port);
        ~LidarScanner();
    
        bool initialize();
        bool getScans(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_);
        // void savePointCloud(const std::string& filename);
    
    private:
        std::string port_;
        int baudrate_;
        rp::standalone::rplidar::RPlidarDriver *lidar_;
        int baudrate = 460800;
        
};