#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0); // Try index 0 first
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::imshow("Camera Test", frame);
        if (cv::waitKey(10) == 27) break; // Exit on 'ESC'
    }
    return 0;
}
