// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp> // For ORB

#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
#include "LidarScanner.h"
#include <atomic>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <termios.h> // For termios functions

std::atomic<bool> interupt(false);  // Atomic flag to safely stop the process

// -------------------- Stereo Image Saving --------------------
std::queue<std::pair<cv::Mat, std::string>> imageQueue;
std::mutex imageMutex;
std::condition_variable imageCondVar;
bool stopImageSaving = false;

// -------------------- PCD Saving --------------------
std::queue<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string>> pcdQueue;
std::mutex pcdMutex;
std::condition_variable pcdCondVar;
bool stopPCDSaving = false;

// -------------------- Ensure Directories Exist --------------------
void createDirectories() {
    std::filesystem::create_directories("data/images/left");  // Left camera images
    std::filesystem::create_directories("data/images/right"); // Right camera images
    std::filesystem::create_directories("data/pcd");   // PCD files
}

// -------------------- Image Saving Thread --------------------
void saveImages() {
    while (true) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });

        if (stopImageSaving && imageQueue.empty()) break;

        if (!imageQueue.empty()) {
            auto imgPair = imageQueue.front();
            imageQueue.pop();
            lock.unlock();

            cv::imwrite(imgPair.second, imgPair.first); // Save image
            // std::cout << "Saved Image: " << imgPair.second << std::endl;

            lock.lock();
        }
    }
}

// -------------------- PCD Saving Thread --------------------
void savePCDFiles() {
    while (true) {
        std::unique_lock<std::mutex> lock(pcdMutex);
        pcdCondVar.wait(lock, [] { return !pcdQueue.empty() || stopPCDSaving; });

        if (stopPCDSaving && pcdQueue.empty()) break;

        if (!pcdQueue.empty()) {
            auto pcdPair = pcdQueue.front();
            pcdQueue.pop();
            lock.unlock();
            if(!pcdPair.first->empty()){;
            pcl::io::savePCDFileBinary(pcdPair.second, *pcdPair.first); // Save PCD
            // std::cout << "Saved PCD: " << pcdPair.second << std::endl;
            }
            lock.lock();
        }
    }
}

// Non-blocking keyboard input function for ESC key press
void listen_for_esc() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);  // Get current terminal settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Apply new settings

    while (true) {
        char ch = getchar(); // Read a single character
        if (ch == 27 || interupt) { // ESC key
            interupt = true;  // Set interrupt flag
            std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old terminal settings
}


void camera_record(StereoCamera stereoCam, VisualOdometry vo, std::vector<cv::Mat>& TotalTransformation){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    
    auto now = std::chrono::steady_clock::now();
    // auto start = std::chrono::steady_clock::now();
    // auto totalTime =  std::chrono::duration_cast<std::chrono::milliseconds>(startTime - start).count();
    int frameCounter = 0;
    // Initial pose
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    // cv::Mat R_matrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    // totalRotation.push_back((cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1));
    // totalTranslation.push_back((cv::Mat_<double>(3, 1) << 0, 0, 0));
    // totalTranslation.push_back((cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1));
    cv::Mat leftFrame_pre, rightFrame_pre, leftFrame, rightFrame;
    cv::Mat identity = cv::Mat::eye(4, 4, CV_32F);
    // Define a vector to store timestamps
    std::vector<double> timestamps;
    TotalTransformation.push_back(identity);
    
    while (!interupt) {
        // cv::Mat leftFrame, rightFrame;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - now).count();
        cv::Mat rel_transform;
        if (elapsed >= 40) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                // Get current time in seconds with microsecond precision
                auto now = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
                double timestamp = duration.count() / 1e6;  // Convert microseconds to seconds
                // Store timestamp
                timestamps.push_back(timestamp);

                std::string leftPath = "data/images/left/left_" + std::to_string(timestamp) + ".png";
                std::string rightPath = "data/images/right/right_" + std::to_string(timestamp) + ".png";

                {
                    std::lock_guard<std::mutex> lock(imageMutex);
                    imageQueue.push({leftFrame, leftPath});
                    imageQueue.push({rightFrame, rightPath});
                }
                imageCondVar.notify_one(); // Notify saving thread

                // startTime = std::chrono::steady_clock::now(); // Reset timer
                
                if (frameCounter > 0){
                    rel_transform = vo.StereoOdometry(leftFrame_pre, leftFrame, rightFrame_pre, rightFrame); //, totalRotation[frameCounter-1], totalTranslation[frameCounter-1]);
                    vo.updatePose(TotalTransformation, rel_transform, frameCounter-1);
                    
                    // totalTranslation.push_back(motionPair.first);
                    // totalRotation.push_back(rel_transform);
                    // std::cout<<"frame Count: "<< frameCounter <<std::endl;
                    // std::cout<<"time diff ms: "<< elapsed <<std::endl;
                    // std::cout<<motionPair.first[2]<<std::endl;
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
                interupt = true;
                break;
            }                   

            frameCounter++;
            // auto curTime = std::chrono::steady_clock::now();
            // auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
            // std::cout<<"frame Count: "<< frameCounter <<std::endl;
            // std::cout<<"time diff ms: "<< totalTime <<std::endl;
            
        }        
        // std::cout << "Elapsed time: " << totalTime << " milliseconds\n";
        
        // if (totalTime >= 7000) // Stop after 1000 frames (adjust as needed)
        //     break;
    }

    // Stop saving thread
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        stopImageSaving = true;
    }
    imageCondVar.notify_one();

    cv::FileStorage fs("../transformations_camera.json", cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

    fs << "TotalTransformation" << "[";
    for (size_t i = 0; i < TotalTransformation.size(); i++) {
        fs << "{";
        fs << "timestamp" << timestamps[i];  // Use the corresponding timestamp
        fs << "matrix" << TotalTransformation[i];
        fs << "}";
    }
    fs << "]";

    fs.release();
    

}

int lidar_record(LidarScanner lidarscan, lidar_odometry lidar_odom, std::vector<cv::Mat>& TotalTransformation){

    if (!lidarscan.initialize()) {
        std::cerr << "RPLIDAR C1 initialization failed!" << std::endl;
        return -1;
    }
    auto now = std::chrono::steady_clock::now();
    // auto start = std::chrono::steady_clock::now();

    // Initialize point cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr scans_pre(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scans_cur(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr scans_pre;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr scans_cur;

    cv::Mat rel_transform;
    int frameCounter = 0;
    // Define a vector to store timestamps
    std::vector<double> timestamps;
    // totalRotation.push_back((cv::Mat::eye(4, 4, CV_32F)))
    // totalRotation.push_back((cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1));
    // totalTranslation.push_back((cv::Mat_<double>(3, 1) << 0, 0, 0));
    // totalTranslation.push_back((cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1));
    cv::Mat identity = cv::Mat::eye(4, 4, CV_32F);
    TotalTransformation.push_back(identity);
    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - now).count();
        
        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            if (lidarscan.getScans(scans_cur)) {

                // Get current time in seconds with microsecond precision
                auto now = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
                double timestamp = duration.count() / 1e6;  // Convert microseconds to seconds
                // Store timestamp
                timestamps.push_back(timestamp);
                
                std::string pcdFilename = "data/pcd/pcd_" + std::to_string(timestamp) + ".pcd";
                {
                    std::lock_guard<std::mutex> lock(pcdMutex);
                    pcdQueue.push({scans_cur, pcdFilename});
                }
                pcdCondVar.notify_one(); // Notify saving thread

                // startTime = std::chrono::steady_clock::now(); // Reset timer
                // std::cout << "Got lidar scan" <<std::endl;
                // std::cout << "frame: " << frameCounter <<std::endl;

                if(frameCounter > 0){

                    rel_transform = lidar_odom.lidar_odom(scans_pre, scans_cur);//, totalRotation[frameCounter-1], totalTranslation[frameCounter-1]);
                    lidar_odom.updatePose(TotalTransformation, rel_transform, frameCounter-1);
                    // totalTranslation.push_back(motionPair.first);
                    // std::cout << "totalTranslation[0]:\n" << totalTranslation[frameCounter-1] << std::endl;
                    // totalRotation.push_back(rel_transform);
                }

                // Deep copy cloud2 into cloud1
                // Ensure scans_cur is properly initialized before copying
                
                scans_pre->clear();
                pcl::copyPointCloud(*scans_cur, *scans_pre);
                scans_cur->clear();
            }
            else {
                std::cerr << "Failed to obtain lidar scans" << std::endl;
                interupt = true;
                break;
            } 

            frameCounter++;
            // auto curTime = std::chrono::steady_clock::now();
            // auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
            // std::cout<<"frame Count lidar: "<< frameCounter <<std::endl;
            // std::cout<<"time lidar ms: "<< totalTime <<std::endl;
        }
        
    }

    {
        std::lock_guard<std::mutex> lock(pcdMutex);
        stopPCDSaving = true;
    }
    pcdCondVar.notify_one();

    cv::FileStorage fs("../transformations_lidar.json", cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

    fs << "TotalTransformation" << "[";
    for (size_t i = 0; i < TotalTransformation.size(); i++) {
        fs << "{";
        fs << "timestamp" << timestamps[i];  // Use the corresponding timestamp
        fs << "matrix" << TotalTransformation[i];
        fs << "}";
    }
    fs << "]";

    fs.release();

    return 0;
}



int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <left_camera_id> <right_camera_id> <lidar_port>" << std::endl;
        return 1;
    }

    // Parse camera IDs from command-line arguments
    int left_camera_id, right_camera_id;
    std::string lidar_port = argv[3];
    std::istringstream(argv[1]) >> left_camera_id;
    std::istringstream(argv[2]) >> right_camera_id;
    // std::string(argv[3]) >> lidar_port;

    StereoCamera stereoCam(left_camera_id, right_camera_id);
    VisualOdometry vo;

    LidarScanner lidarscan(lidar_port);
    lidar_odometry lidar_odom;

    std::vector<cv::Mat> TotalTransformation_camera;

    std::vector<cv::Mat> TotalTransformation_lidar;
    createDirectories(); // Ensure save directories exist

    // camera_record(stereoCam, vo, totalTranslation, totalRotation);

    // Launching user input in a separate thread
    std::thread inputThread(listen_for_esc);

    // Start image-saving thread
    std::thread saveImageThread(saveImages);

    // Start pcd-saving thread
    std::thread savePCDThread(savePCDFiles);

    // Start the camera recording threads
    std::thread cameraThread(camera_record, stereoCam, vo, std::ref(TotalTransformation_camera));

    // Start the pcd recording threads
    std::thread lidarThread(lidar_record, lidarscan, lidar_odom, std::ref(TotalTransformation_lidar));

    // Wait for the recording threads to finish
    lidarThread.join();
    cameraThread.join();
    saveImageThread.join();
    savePCDThread.join();
    // Join the user input thread (this thread will wait for "stop" command)
    inputThread.join();

    
    

    

    // std::cout<<totalTranslation<<std::endl;
    // std::thread t1(camera_record, stereoCam, vo, &totalTranslation, &totalRotation);
    // std::thread t2(lidar_record, lidar);

    // t1.join();
    // t2.join();
    std::cout<< "Recording complete"<<"\n";
    return 0;
}
