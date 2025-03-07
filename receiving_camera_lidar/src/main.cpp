#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <filesystem>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <queue>
#include <mutex>
#include <termios.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 5005
#define PACKET_SIZE 4096

// Atomic flag to safely stop the process
std::atomic<bool> interrupt(false);

// -------------------- Queues and Synchronization --------------------
std::queue<std::pair<cv::Mat, std::string>> imageQueue;
std::mutex imageMutex;
std::condition_variable imageCondVar;
std::atomic<bool> stopImageSaving(false);

std::queue<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string>> pcdQueue;
std::mutex pcdMutex;
std::condition_variable pcdCondVar;
std::atomic<bool> stopPCDSaving(false);

// -------------------- Directory Management --------------------
void createDirectories() {
    std::filesystem::create_directories("data/images/left");
    std::filesystem::create_directories("data/images/right");
    std::filesystem::create_directories("data/pcd");
}

// -------------------- Image Saving Thread --------------------
void saveImages() {
    while (!interrupt) {
        std::unique_lock<std::mutex> lock(imageMutex);
        imageCondVar.wait(lock, [] { return !imageQueue.empty() || stopImageSaving; });

        if (stopImageSaving && imageQueue.empty()) break;

        if (!imageQueue.empty()) {
            auto imgPair = imageQueue.front();
            imageQueue.pop();
            lock.unlock();
            cv::imwrite(imgPair.second, imgPair.first);
            lock.lock();
        }
    }
}

// -------------------- PCD Saving Thread --------------------
void savePCDFiles() {
    while (!interrupt) {
        std::unique_lock<std::mutex> lock(pcdMutex);
        pcdCondVar.wait(lock, [] { return !pcdQueue.empty() || stopPCDSaving; });

        if (stopPCDSaving && pcdQueue.empty()) break;

        if (!pcdQueue.empty()) {
            auto pcdPair = pcdQueue.front();
            pcdQueue.pop();
            lock.unlock();
            if (!pcdPair.first->empty()) {
                pcl::io::savePCDFileBinary(pcdPair.second, *pcdPair.first);
            }
            lock.lock();
        }
    }
}

// -------------------- Non-blocking Keyboard Input --------------------
void listenForEsc() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (true) {
        char ch = getchar();
        if (ch == 27 || interrupt) { // ESC key
            interrupt = true;
            stopPCDSaving = true;
            stopImageSaving = true;
            std::cout << "Esc key pressed. Interrupt signal sent!" << std::endl;
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// -------------------- Data Processing --------------------
std::pair<int, int> extractDimensions(const std::string& dataDim) {
    size_t xPos = dataDim.find('x');
    if (xPos == std::string::npos) {
        std::cerr << "Invalid dimension format!" << std::endl;
        return {0, 0};
    }
    int height = std::stoi(dataDim.substr(0, xPos));
    int width = std::stoi(dataDim.substr(xPos + 1));
    return {height, width};
}

void processImage(cv::Mat image, const std::string& timestamp, const std::string& type) {
    if (image.empty()) {
        std::cerr << "Failed to decode image data!" << std::endl;
        return;
    }
    std::string typeImage = " ";
    if(type == "L"){
        typeImage = "left";
    }
    else{
        typeImage = "right";
    }

    std::string path = "data/images/" + typeImage + "/" + type + "_" + timestamp + ".png";
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        imageQueue.push({image, path});
    }
    imageCondVar.notify_one();
}

void processLidar(cv::Mat lidarMatrix, const std::string& timestamp) {
    if (lidarMatrix.empty()) {
        std::cerr << "Failed to decode lidar data!" << std::endl;
        return;
    }

    if (lidarMatrix.cols != 2) {
        std::cerr << "Invalid matrix format: Expected 2 columns for (X, Y)" << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scans(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < lidarMatrix.rows; i++) {
        float x = lidarMatrix.at<float>(i, 0);
        float y = lidarMatrix.at<float>(i, 1);
        scans->points.push_back(pcl::PointXYZ(x, y, 0.0));
    }

    std::string pcdFilename = "data/pcd/pcd_" + timestamp + ".pcd";
    {
        std::lock_guard<std::mutex> lock(pcdMutex);
        pcdQueue.push({scans, pcdFilename});
    }
    pcdCondVar.notify_one();
}

// -------------------- TCP Data Reception --------------------
void receiveData() {
    int serverSock = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSock < 0) {
        perror("Socket creation failed");
        return;
    }

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("Bind failed");
        close(serverSock);
        return;
    }

    if (listen(serverSock, 5) < 0) {
        perror("Listen failed");
        close(serverSock);
        return;
    }

    std::cout << "Waiting for TCP connections...\n";

    struct sockaddr_in clientAddr;
    socklen_t clientLen = sizeof(clientAddr);
    int clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &clientLen);
    if (clientSock < 0) {
        perror("Accept failed");
        close(serverSock);
        return;
    }

    while (!interrupt) {
        size_t headerSize;
        recv(clientSock, &headerSize, sizeof(headerSize), 0);

        std::vector<char> headerBuffer(headerSize);
        recv(clientSock, headerBuffer.data(), headerSize, 0);
        std::string receivedHeader(headerBuffer.begin(), headerBuffer.end());

        size_t pos1 = receivedHeader.find('|');
        size_t pos2 = receivedHeader.find('|', pos1 + 1);
        // size_t pos3 = receivedHeader.find('|', pos2 + 1);

        if (pos1 == std::string::npos || pos2 == std::string::npos ) {
            std::cerr << "Invalid data format!" << std::endl;
            // return;
        }

        std::string type = receivedHeader.substr(0, pos1);
        std::string timestamp = receivedHeader.substr(pos1 + 1, pos2 - pos1 - 1);
        // std::string dataDim = receivedHeader.substr(pos2 + 1, pos3 - pos2 - 1);

        size_t imageSize;
        recv(clientSock, &imageSize, sizeof(imageSize), 0);

        std::vector<uchar> buffer(imageSize);
        size_t receivedBytes = 0;
        while (receivedBytes < imageSize) {
            ssize_t recvSize = recv(clientSock, buffer.data() + receivedBytes, imageSize - receivedBytes, 0);
            if (recvSize < 0) {
                perror("Error receiving data");
                close(clientSock);
                return;
            }
            receivedBytes += recvSize;
        }

        cv::Mat data = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if (data.empty()) {
            std::cerr << "Failed to decode received image\n";
            return;
        }

        if (type == "R" || type == "L") {
            processImage(data, timestamp, type);
        } else if (type == "D") {
            processLidar(data, timestamp);
        } else {
            std::cerr << "Unknown header type!" << std::endl;
        }
    }

    close(clientSock);
    close(serverSock);
}

// -------------------- Main Function --------------------
int main() {
    createDirectories();

    std::thread inputThread(listenForEsc);
    std::thread saveImageThread(saveImages);
    std::thread savePCDThread(savePCDFiles);
    std::thread receiveThread(receiveData);

    receiveThread.join();
    saveImageThread.join();
    savePCDThread.join();
    inputThread.join();

    std::cout << "Recording complete\n";
    return 0;
}