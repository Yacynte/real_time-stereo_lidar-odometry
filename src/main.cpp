#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> // For ORB
#include <pcl/io/pcd_io.h>
#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
#include "LiDAR.h"

bool interupt = false;
// #include <filesystem> // For creating folders (C++17)

void camera_record(StereoCamera stereoCam, VisualOdometry vo, 
                    std::vector<std::vector<double>>& totalTranslation, std::vector<cv::Mat>& totalRotation){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    int frameCounter = 0;
    cv::Mat leftFrame_pre, rightFrame_pre, leftFrame, rightFrame;
    while (!interupt) {
        // cv::Mat leftFrame, rightFrame;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                cv::imwrite("../data/left/left_" + std::to_string(frameCounter) + ".png", leftFrame);
                cv::imwrite("../data/right/right_" + std::to_string(frameCounter) + ".png", rightFrame);
                if (frameCounter > 0){
                    std::pair<std::vector<double>, cv::Mat> motionPair;
                    std::cout<<"before Odom: "<< frameCounter <<std::endl;
                    motionPair = vo.StereoOdometry(leftFrame_pre, leftFrame, rightFrame_pre, rightFrame);
                    totalTranslation.push_back(motionPair.first);
                    totalRotation.push_back(motionPair.second);
                    std::cout<<motionPair.first[2]<<std::endl;
                }
                // Update the pre-frame values here **after** processing
                leftFrame_pre = leftFrame.clone();
                rightFrame_pre = rightFrame.clone();

                // std::cout << "FrameCounter: " << frameCounter 
                //         << ", LeftFrame: " << !leftFrame.empty()
                //         << ", RightFrame: " << !rightFrame.empty()
                //         << ", LeftFrame_Pre: " << !leftFrame_pre.empty()
                //         << ", RightFrame_Pre: " << !rightFrame_pre.empty()
                //         << std::endl;
            } 
            else {
                std::cerr << "Failed to capture stereo frames" << std::endl;
            }                   

            frameCounter++;
            startTime = std::chrono::steady_clock::now(); // Reset timer
        }
        auto curTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
        if (totalTime >= 4000) // Stop after 1000 frames (adjust as needed)
            break;
    }

}

void lidar_record(LiDAR lidar){

    cv::Mat leftFrame, rightFrame;
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    int frameCounter = 0;

    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture LiDAR point cloud
            auto pointCloud = lidar.capturePointCloud();
            pcl::io::savePCDFileBinary("data/lidar/pointcloud_" + std::to_string(frameCounter) + ".pcd", *pointCloud);

            frameCounter++;
            startTime = std::chrono::steady_clock::now(); // Reset timer
        }
        auto curTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
        if (totalTime >= 50) // Stop after 1000 frames (adjust as needed)
            break;
    }

}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <left_camera_id> <right_camera_id>" << std::endl;
        return 1;
    }

    // Parse camera IDs from command-line arguments
    int left_camera_id, right_camera_id;
    std::istringstream(argv[1]) >> left_camera_id;
    std::istringstream(argv[2]) >> right_camera_id;

    StereoCamera stereoCam(left_camera_id, right_camera_id);
    LiDAR lidar;
    VisualOdometry vo;

    std::vector<cv::Mat> totalRotation;
    std::vector<std::vector<double>> totalTranslation;
    camera_record(stereoCam, vo, totalTranslation, totalRotation);

    
    cv::FileStorage fs("../transformations.json", cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

    fs << "TotalRotation" << "[";
    for (const auto& R : totalRotation) {
        fs << R;
    }
    fs << "]";

    fs << "TotalTranslation" << "[";
    for (const auto& T : totalTranslation) {
        fs << cv::Mat(T).t(); // Convert vector to Mat for saving
    }
    fs << "]";

    fs.release();

    // std::cout<<totalTranslation<<std::endl;
    // std::thread t1(camera_record, stereoCam, vo, &totalTranslation, &totalRotation);
    // std::thread t2(lidar_record, lidar);

    // t1.join();
    // t2.join();
    std::cout<< "Recording complete"<<"\n";
    return 0;
}
