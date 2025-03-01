
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/common/io.h>  // For copyPointCloud
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <thread>
#include <iostream>
#include <algorithm> 
#include <chrono>
#include <atomic>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <termios.h> // For termios functions

std::atomic<bool> interupt(false);  // Atomic flag to safely stop the process

#define UDP_PORT 5005
// #define BUFFER_SIZE 4096
#define CHUNK_SIZE 4096


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


int setupSocket() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(UDP_PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    return sock;
}

// -------------------- Ensure Directories Exist --------------------
void createDirectories() {
    std::filesystem::create_directories("data/images/left");  // Left camera images
    std::filesystem::create_directories("data/images/right"); // Right camera images
    std::filesystem::create_directories("data/pcd");   // PCD files
}

void receiveImage(int sock, const std::string& label, const std::string& timestampStr) {

    sockaddr_in senderAddr;
    socklen_t senderAddrLen = sizeof(senderAddr);
    // Receive the image in chunks
    std::vector<unsigned char> imageData;
    char buffer[CHUNK_SIZE];

    while (true) {
        int bytesReceived = recvfrom(sock, buffer, CHUNK_SIZE, 0, (struct sockaddr*)&senderAddr, &senderAddrLen);
        if (bytesReceived <= 0) {
            std::cerr << "Error receiving image data" << std::endl;
            break;
        }

        // Check for "END" marker
        if (bytesReceived == 3 && std::memcmp(buffer, "END", 3) == 0) {
            // std::cout << "End of Image Data" << std::endl;
            break;
        }

        // Append to image buffer
        imageData.insert(imageData.end(), buffer, buffer + bytesReceived);
    }

    // Decode and save the image
    cv::Mat image = cv::imdecode(imageData, cv::IMREAD_COLOR);
    if (!image.empty()) {
        std::string filename = (label == "L" ? "data/images/left/left_" : "data/images/right/right_") + timestampStr + ".png";
        cv::imwrite(filename, image);
        // std::cout << "Saved " << filename << std::endl;
    } else {
        std::cerr << "Error decoding image" << std::endl;
    }
}

// Function to receive a PCD point cloud
void receivePointCloud(int sock, const std::string& timestamp) {
    std::vector<uint8_t> serializedData;
    // serializedData.reserve(dataSize);

    // std::vector<unsigned char> imageData;
    sockaddr_in senderAddr;
    socklen_t senderAddrLen = sizeof(senderAddr);

    char buffer[CHUNK_SIZE];
    while (true) {
        int bytesReceived = recvfrom(sock, buffer, CHUNK_SIZE, 0, (struct sockaddr*)&senderAddr, &senderAddrLen);
        if (bytesReceived <= 0) {
            std::cerr << "Error receiving PCD data" << std::endl;
            break;
        }

        // Check for "END" marker
        if (bytesReceived == 3 && std::memcmp(buffer, "END", 3) == 0) {
            // std::cout << "End of PCD Data" << std::endl;
            break;
        }

        // Append to image buffer
        serializedData.insert(serializedData.end(), buffer, buffer + bytesReceived);
    }

     // Write the received bytes to a temporary file
    std::string tempFilename = "temp" + timestamp + ".pcd";
    std::ofstream tempFile(tempFilename, std::ios::binary);
    tempFile.write(reinterpret_cast<char*>(serializedData.data()), serializedData.size());
    tempFile.close();

    // Load the point cloud from the temporary file
    pcl::PointCloud<pcl::PointXYZ>::Ptr receivedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    if (reader.read(tempFilename, *receivedCloud) == -1) {
        std::cerr << "Error reading PCD file: " << tempFilename << std::endl;
        return;
    }
 
     // Save the received point cloud
     std::string filename = "pcd" + timestamp + ".pcd";
     pcl::io::savePCDFileBinary(filename, *receivedCloud);
    //  std::cout << "Saved received PCD: " << filename << std::endl;

}


int main() {
    int sock = setupSocket();
    sockaddr_in senderAddr;
    socklen_t senderAddrLen = sizeof(senderAddr);


    while (!interupt) {

        char headerBuffer[100];  // Buffer for the header
        int headerBytes = recvfrom(sock, headerBuffer, sizeof(headerBuffer) - 1, 0, (struct sockaddr*)&senderAddr, &senderAddrLen);
        if (headerBytes <= 0) {
            std::cerr << "Failed to receive header" << std::endl;
            continue;
        }
        headerBuffer[headerBytes] = '\0';  // Null-terminate

        // Parse the header: "L|1710001234567|"
        std::string header(headerBuffer);
        size_t firstPipe = header.find('|');
        size_t secondPipe = header.find('|', firstPipe + 1);

        if (firstPipe == std::string::npos || secondPipe == std::string::npos) {
            std::cerr << "Invalid header format" << std::endl;
            continue;
        }

        std::string label = header.substr(0, firstPipe);  // "L" or "R"
        std::string timestampStr = header.substr(firstPipe + 1, secondPipe - firstPipe - 1);
        uint64_t timestamp = std::stoull(timestampStr);  // Convert to integer

        std::cout << "Receiving " << label << " image at " << timestamp << std::endl;


        std::thread imageThread, lidarThread;
        if (label == "L" || label == "R") {
            imageThread = std::thread(receiveImage, sock, label, timestampStr);
        } else {
            lidarThread = std::thread(receivePointCloud, sock, timestampStr);
        }

        if (imageThread.joinable()) imageThread.join();
        if (lidarThread.joinable()) lidarThread.join();
    
    }

    close(sock);
    return 0;
}
