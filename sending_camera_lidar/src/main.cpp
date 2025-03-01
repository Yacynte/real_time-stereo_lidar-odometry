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


#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>


#define UDP_IP "192.168.178.28"
#define UDP_PORT 5005
#define CHUNK_SIZE 4096

std::atomic<bool> interupt(false);  // Atomic flag to safely stop the process


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

// Function to get the current timestamp in milliseconds
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return std::to_string(duration.count());
}

// Function to send an image
void sendImage(int sock, struct sockaddr_in serverAddr, cv::Mat& image, const std::string& label, std::string timestamp) {
    std::vector<uchar> buffer;
    cv::imencode(".jpg", image, buffer, {cv::IMWRITE_JPEG_QUALITY, 80});

    // std::string timestamp = getTimestamp();
    std::string header = label + "|" + timestamp + "|";  // Example: "L|1710001234567|"
    sendto(sock, header.c_str(), header.size(), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    usleep(10000);

    // Send image data in chunks
    for (size_t i = 0; i < buffer.size(); i += CHUNK_SIZE) {
        sendto(sock, buffer.data() + i, std::min(static_cast<size_t>(CHUNK_SIZE), buffer.size() - i), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    }

    sendto(sock, "END", 3, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
}

// Function to serialize Point Cloud data
std::vector<unsigned char> serializePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PCDWriter writer;
    pcl::PCLPointCloud2 cloudBlob;
    
    // Convert to PCLPointCloud2 format
    pcl::toPCLPointCloud2(*cloud, cloudBlob);

    // Write to a temporary file
    std::string tempFile = "/tmp/temp_cloud.pcd";
    if (writer.writeBinaryCompressed(tempFile, cloudBlob) != 0) {
        throw std::runtime_error("Failed to write point cloud to file.");
    }

    // Read the file into a buffer
    std::ifstream file(tempFile, std::ios::binary | std::ios::ate);
    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> buffer(fileSize);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize)) {
        throw std::runtime_error("Failed to read point cloud file.");
    }

    return buffer;
}


// Function to send serialized PCD over UDP
void sendPointCloud(int sock, struct sockaddr_in serverAddr, pcl::PointCloud<pcl::PointXYZ>::Ptr scans_cur, std::string timestamp) {
    std::vector<uint8_t> serializedData = serializePointCloud(scans_cur);
    // std::string timestamp = getTimestamp();

    // Send metadata first
    std::string header = "P|" + timestamp + "|" + std::to_string(serializedData.size()) + "|";
    sendto(sock, header.c_str(), header.size(), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    usleep(10000);  // Small delay to avoid packet loss

    // Send data in chunks
    size_t offset = 0;
    while (offset < serializedData.size()) {
        size_t chunkSize = std::min(static_cast<size_t>(CHUNK_SIZE), serializedData.size() - offset);
        sendto(sock, serializedData.data() + offset, chunkSize, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
        offset += chunkSize;
    }

    sendto(sock, "END", 3, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    std::cout << "Sent PCD scan with timestamp " << timestamp << std::endl;
}

void camera_record(StereoCamera stereoCam, int sock, struct sockaddr_in serverAddr){
    // StereoCamera stereoCam(0, 2); // Adjust IDs based on your setup

    
    auto now = std::chrono::steady_clock::now();
    int frameCounter = 0;

    cv::Mat leftFrame, rightFrame;
    // Define a vector to store timestamps
    
    while (!interupt) {
        // cv::Mat leftFrame, rightFrame;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - now).count();
        if (elapsed >= 40) { // 1000 ms / 20 FPS = 50 ms
            // Capture stereo frames
            if (stereoCam.captureFrames(leftFrame, rightFrame)) {
                // Get current time in seconds with microsecond precision
                std::string timestamp = getTimestamp();

                std::thread leftThread(sendImage, sock, serverAddr, std::ref(leftFrame), "L", timestamp);
                std::thread rightThread(sendImage, sock, serverAddr, std::ref(rightFrame), "R", timestamp);
                           
                leftThread.join();
                rightThread.join();
            }        
        }
    }
}


    
void lidar_record(LidarScanner lidarscan, int sock, struct sockaddr_in serverAddr){

    if (!lidarscan.initialize()) {
        std::cerr << "RPLIDAR C1 initialization failed!" << std::endl;
        return ;
    }
    auto now = std::chrono::steady_clock::now();
    // auto start = std::chrono::steady_clock::now();

    // Initialize point cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr scans_cur(new pcl::PointCloud<pcl::PointXYZ>);

    while (!interupt) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - now).count();
        
        if (elapsed >= 50) { // 1000 ms / 20 FPS = 50 ms
            if (lidarscan.getScans(scans_cur)) {

                // Get current time in seconds with microsecond precision
                std::string timestamp = getTimestamp();
                
                sendPointCloud(sock, serverAddr, scans_cur, timestamp);
                scans_cur->clear();
            }
            else {
                std::cerr << "Failed to obtain lidar scans" << std::endl;
                interupt = true;
                break;
            } 

        }
        
    }

}



int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <left_camera_id> <right_camera_id> <lidar_port>" << std::endl;
        return 1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_IP, &serverAddr.sin_addr);

    // Parse camera IDs from command-line arguments
    int left_camera_id, right_camera_id;
    std::string lidar_port = argv[3];
    std::istringstream(argv[1]) >> left_camera_id;
    std::istringstream(argv[2]) >> right_camera_id;
    // std::string(argv[3]) >> lidar_port;

    StereoCamera stereoCam(left_camera_id, right_camera_id);

    LidarScanner lidarscan(lidar_port);

    // camera_record(stereoCam, vo, totalTranslation, totalRotation);

    // Launching user input in a separate thread
    std::thread inputThread(listen_for_esc);

    // Start the camera recording threads
    std::thread cameraThread(camera_record, stereoCam, sock, serverAddr);

    // Start the pcd recording threads
    std::thread lidarThread(lidar_record, lidarscan, sock, serverAddr);

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
