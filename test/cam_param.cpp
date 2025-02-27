#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open stereo camera (single input providing side-by-side images)
    cv::VideoCapture cap(2, cv::CAP_V4L2);  
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // Set the stereo frame resolution (side-by-side)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    // Capture a single frame
    cv::Mat frame;
    cap >> frame;
    cap.release();  // Release the camera

    if (frame.empty()) {
        std::cerr << "Error: Could not capture an image." << std::endl;
        return -1;
    }

    // Get original stereo image dimensions
    int stereoWidth = frame.cols;  // 3840
    int stereoHeight = frame.rows; // 1080
    int width = stereoWidth / 2;   // Each camera's width (1920)
    int height = stereoHeight;     // Each camera's height (1080)

    // Estimate intrinsic parameters for each individual camera
    double fx = 0.5 * width;  // Approximate focal length
    double fy = 0.5 * height;
    double cx = width / 2.0;
    double cy = height / 2.0;

    // Construct the intrinsic matrix for each camera
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                            0, fy, cy,
                                            0,  0,  1);

    // Print original intrinsics
    std::cout << "Original Stereo Frame Resolution: " << stereoWidth << "x" << stereoHeight << std::endl;
    std::cout << "Individual Camera Resolution: " << width << "x" << height << std::endl;
    std::cout << "Estimated Camera Intrinsic Matrix (Before Resizing):\n" << K << std::endl;

    // Resize to 640×480
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, cv::Size(1280, 480)); // 640x480 per camera

    // Scale intrinsics accordingly
    double scaleX = 640.0 / width;
    double scaleY = 480.0 / height;
    fx *= scaleX;
    fy *= scaleY;
    cx *= scaleX;
    cy *= scaleY;

    // New intrinsic matrix after resizing
    cv::Mat K_resized = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                                    0, fy, cy,
                                                    0,  0,  1);

    std::cout << "Resized Individual Camera Resolution: 640x480" << std::endl;
    std::cout << "Updated Camera Intrinsic Matrix (After Resizing):\n" << K_resized << std::endl;

    return 0;
}
