#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> // For ORB
#include <pcl/io/pcd_io.h>
#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
#include "LiDAR.h"
#include <atomic>
#include <unistd.h>  // For _getch() (on Unix-like systems)
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
        if (ch == 27) { // ESC key
            interupt = true;  // Set interrupt flag
            std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old terminal settings
}

void camera_record(StereoCamera stereoCam, VisualOdometry vo, 
                    std::vector<std::vector<double>>& totalTranslation, std::vector<cv::Mat>& totalRotation){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    
    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    // auto totalTime =  std::chrono::duration_cast<std::chrono::milliseconds>(startTime - start).count();
    int frameCounter = 0;
    cv::Mat leftFrame_pre, rightFrame_pre, leftFrame, rightFrame;
    while (!interupt) {
        // cv::Mat leftFrame, rightFrame;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        
        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                startTime = std::chrono::steady_clock::now(); // Reset timer
                cv::imwrite("../data/left/left_" + std::to_string(frameCounter) + ".png", leftFrame);
                cv::imwrite("../data/right/right_" + std::to_string(frameCounter) + ".png", rightFrame);
                if (frameCounter > 0){
                    std::pair<std::vector<double>, cv::Mat> motionPair;
                    motionPair = vo.StereoOdometry(leftFrame_pre, leftFrame, rightFrame_pre, rightFrame);
                    totalTranslation.push_back(motionPair.first);
                    totalRotation.push_back(motionPair.second);
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
                break;
            }                   

            frameCounter++;
            
        }        
        // std::cout << "Elapsed time: " << totalTime << " milliseconds\n";
        
        // if (totalTime >= 7000) // Stop after 1000 frames (adjust as needed)
        //     break;
    }
    auto curTime = std::chrono::steady_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
    std::cout<<"frame Count: "<< frameCounter <<std::endl;
    std::cout<<"time diff ms: "<< totalTime <<std::endl;

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
    // LiDAR lidar;
    VisualOdometry vo;

    std::vector<cv::Mat> totalRotation;
    std::vector<std::vector<double>> totalTranslation;
    // camera_record(stereoCam, vo, totalTranslation, totalRotation);

    // Launching user input in a separate thread
    std::thread inputThread(listen_for_esc);

    // Start the recording threads
    std::thread cameraThread(camera_record, stereoCam, vo, std::ref(totalTranslation), std::ref(totalRotation));
    // std::thread lidarThread(lidar_record, lidar);

    // Wait for the recording threads to finish
    cameraThread.join();
    // lidarThread.join();

    // Join the user input thread (this thread will wait for "stop" command)
    inputThread.join();

    
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
