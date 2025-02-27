// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp> // For ORB

#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
#include "LidarScanner.h"
#include <atomic>
// #include <unistd.h>  // For _getch() (on Unix-like systems)
#include <termios.h> // For termios functions

std::atomic<bool> interupt(false);  // Atomic flag to safely stop the process

// Function to handle user input for interrupting the process
// void user_input() {
//     std::string userCommand;
//     while (true) {
//         // std::getline(std::cin, userCommand);
//         // if (userCommand == "stop") {
//         //     interupt = true; // Set the flag to true when the user inputs "stop"
//         //     break;
//         // }
//         char key;
//         if (std::cin >> key) {
//             if (key == 27) { // Check if Esc key is pressed (ASCII code 27)
//                 interupt = true;
//                 std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
//                 break;
//             }
//         }
//     }
// }

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

void camera_record(StereoCamera stereoCam, VisualOdometry vo, std::vector<cv::Mat>& totalTranslation, std::vector<cv::Mat>& totalRotation){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
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
    totalTranslation.push_back(identity);
    
    while (!interupt) {
        // cv::Mat leftFrame, rightFrame;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        cv::Mat rel_transform;
        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                startTime = std::chrono::steady_clock::now(); // Reset timer
                cv::imwrite("../data/left/left_" + std::to_string(frameCounter) + ".png", leftFrame);
                cv::imwrite("../data/right/right_" + std::to_string(frameCounter) + ".png", rightFrame);
                if (frameCounter > 0){
                    rel_transform = vo.StereoOdometry(leftFrame_pre, leftFrame, rightFrame_pre, rightFrame); //, totalRotation[frameCounter-1], totalTranslation[frameCounter-1]);
                    vo.updatePose(totalTranslation, rel_transform, frameCounter-1);
                    
                    // totalTranslation.push_back(motionPair.first);
                    totalRotation.push_back(rel_transform);
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
            auto curTime = std::chrono::steady_clock::now();
            auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
            std::cout<<"frame Count: "<< frameCounter <<std::endl;
            // std::cout<<"time diff ms: "<< totalTime <<std::endl;
            
        }        
        // std::cout << "Elapsed time: " << totalTime << " milliseconds\n";
        
        // if (totalTime >= 7000) // Stop after 1000 frames (adjust as needed)
        //     break;
    }

    cv::FileStorage fs("../transformations_camera.json", cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

    fs << "TotalRotation" << "[";
    for (const auto& R : totalRotation) {
        fs << R;
    }
    fs << "]";

    fs << "TotalTranslation" << "[";
    for (const auto& T : totalTranslation) {
        fs << T; // Convert vector to Mat for saving
    }
    fs << "]";

    fs.release();
    

}

int lidar_record(LidarScanner lidarscan, lidar_odometry lidar_odom, std::vector<cv::Mat>& totalTranslation, std::vector<cv::Mat>& totalRotation){

    if (!lidarscan.initialize()) {
        std::cerr << "RPLIDAR C1 initialization failed!" << std::endl;
        return -1;
    }
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();

    // Initialize point cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr scans_pre(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scans_cur(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr scans_pre;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr scans_cur;

    cv::Mat rel_transform;
    int frameCounter = 0;
    // totalRotation.push_back((cv::Mat::eye(4, 4, CV_32F)))
    // totalRotation.push_back((cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1));
    // totalTranslation.push_back((cv::Mat_<double>(3, 1) << 0, 0, 0));
    // totalTranslation.push_back((cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1));
    cv::Mat identity = cv::Mat::eye(4, 4, CV_32F);
    totalTranslation.push_back(identity);
    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        
        if (elapsed >= 100) { // 1000 ms / 20 FPS = 50 ms
            if (lidarscan.getScans(scans_cur)) {
                startTime = std::chrono::steady_clock::now(); // Reset timer
                // std::cout << "Got lidar scan" <<std::endl;
                // std::cout << "frame: " << frameCounter <<std::endl;

                if(frameCounter > 0){

                    rel_transform = lidar_odom.lidar_odom(scans_pre, scans_cur);//, totalRotation[frameCounter-1], totalTranslation[frameCounter-1]);
                    lidar_odom.updatePose(totalTranslation, rel_transform, frameCounter-1);
                    // totalTranslation.push_back(motionPair.first);
                    // std::cout << "totalTranslation[0]:\n" << totalTranslation[frameCounter-1] << std::endl;
                    totalRotation.push_back(rel_transform);
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
            auto curTime = std::chrono::steady_clock::now();
            auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
            std::cout<<"frame Count lidar: "<< frameCounter <<std::endl;
            // std::cout<<"time lidar ms: "<< totalTime <<std::endl;
        }
        
    }
    cv::FileStorage gs("../transformations_lidar.json", cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

    gs << "TotalRotation" << "[";
    for (const auto& R : totalRotation) {
        gs << R;
    }
    gs << "]";

    gs << "TotalTranslation" << "[";
    for (const auto& T : totalTranslation) {
        gs << T; // Convert vector to Mat for saving
    }
    gs << "]";

    gs.release();

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

    std::vector<cv::Mat> totalRotation_camera;
    std::vector<cv::Mat> totalTranslation_camera;

    std::vector<cv::Mat> totalRotation_lidar;
    std::vector<cv::Mat> totalTranslation_lidar;
    // camera_record(stereoCam, vo, totalTranslation, totalRotation);

    // Launching user input in a separate thread
    std::thread inputThread(listen_for_esc);

    // Start the recording threads
    std::thread cameraThread(camera_record, stereoCam, vo, std::ref(totalTranslation_camera), std::ref(totalRotation_camera));
    std::thread lidarThread(lidar_record, lidarscan, lidar_odom, std::ref(totalTranslation_lidar), std::ref(totalRotation_lidar));

    // Wait for the recording threads to finish
    lidarThread.join();
    cameraThread.join();

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
