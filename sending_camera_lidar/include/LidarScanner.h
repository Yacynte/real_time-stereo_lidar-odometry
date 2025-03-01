// LiDAR.h
#pragma once
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "rplidar.h"


using namespace rp::standalone::rplidar;


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
        
};