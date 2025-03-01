// StereoCamera.cpp
#include "StereoCamera.h"
#include <thread>
// Constructor: Open both left and right cameras
StereoCamera::StereoCamera(int leftCamID, int rightCamID) {
    leftCamIDp = leftCamID;
    rightCamIDp = rightCamID;
    leftCam.open(leftCamID, cv::CAP_V4L2);

    // Verify resolution
    // double width = leftCam.get(cv::CAP_PROP_FRAME_WIDTH);
    // double height = leftCam.get(cv::CAP_PROP_FRAME_HEIGHT);

    if (rightCamID == 100){
        // Set resolution to 3840x1080 (side-by-side stereo)
        leftCam.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
        leftCam.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        
    }
    else{
        rightCam.open(rightCamID, cv::CAP_V4L2);
    }

    // if (!checkCameras()) {
    //     std::cerr << "Failed to open one or both cameras." << std::endl;
    // }
}

// Destructor: Release camera resources when the object goes out of scope
StereoCamera::~StereoCamera() {
    leftCam.release();
    if (rightCamIDp == 100){rightCam.release();}
    
}

void StereoCamera::splitStereoImage(cv::Mat &leftFrame, cv::Mat &rightFrame) {
    cv::Mat frame;
    leftCam.read(frame);  // Capture a frame
    // Check if frame is empty
    if (!frame.empty()) {
        // Define left and right ROI
        // std::cout << "Resolution: " << frame.cols << "x" << frame.rows << std::endl;
        cv::Rect leftROI(0, 0, frame.cols / 2, frame.rows);
        cv::Rect rightROI(frame.cols / 2, 0, frame.cols / 2, frame.rows);
    
        // Extract left and right images
        cv::Mat frame1; cv::Mat frame2;

        frame1 = frame(leftROI).clone();
        frame2 = frame(rightROI).clone();
        
        cv::resize(frame1, leftFrame, cv::Size(640, 480));
        cv::resize(frame2, rightFrame, cv::Size(640, 480));
    
        // Show images
        // cv::imshow("Left Image", leftFrame);
        // cv::imshow("Right Image", rightFrame);
        // cv::waitKey(0);
    }
    // cv::imshow("Right Frame", frame);
    //     cv::waitKey(0);  // Wait for a key press
    else {
        std::cerr << "Error: Captured frame is empty!" << std::endl;
        return;
    }
}

// Captures a stereo pair of frames
bool StereoCamera::captureFrames(cv::Mat& leftFrame, cv::Mat& rightFrame) {
    if (!checkCameras()) {
        return false;
    }

    if (rightCamIDp == 100){
        splitStereoImage(leftFrame, rightFrame);
        RectifyImage(leftFrame, rightFrame);
        // auto rectifiedCur = RectifyImage(leftImage_cur, rightImage_cur);

        // leftImageRec_pre = rectifiedPre.first;
        // rightImageRec_pre = rectifiedPre.second;

        return true;
    }
    // Read a frame from both the left and right cameras
    if (!leftCam.read(leftFrame)) {
        std::cerr << "Error: Failed to capture left frame." << std::endl;
        return false;
    }

    if (!rightCam.read(rightFrame)) {
        std::cerr << "Error: Failed to capture right frame." << std::endl;
        return false;
    }

    RectifyImage(leftFrame, rightFrame);
       
    return true;
}

// Checks if both cameras are opened successfully
bool StereoCamera::checkCameras() {
    if (!leftCam.isOpened()) {
        std::cerr << "Error: Could not open left camera stream." << std::endl;
    }

    if (rightCamIDp == 100){
        return leftCam.isOpened();
    }

    if (!rightCam.isOpened()) {
        std::cerr << "Error: Could not open right camera stream." << std::endl;
    }

    return leftCam.isOpened() && rightCam.isOpened();
}

void StereoCamera::RectifyImage( cv::Mat& leftImage, cv::Mat& rightImage) {
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, leftImage.size(), R, T, R1, R2, P1, P2, Q);

    // Compute rectification maps
    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, leftImage.size(), CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, rightImage.size(), CV_32FC1, map2x, map2y);

    // Apply the rectification maps
    cv::Mat rectifiedLeft, rectifiedRight;
    cv::remap(leftImage, rectifiedLeft, map1x, map1y, cv::INTER_LINEAR);
    cv::remap(rightImage, rectifiedRight, map2x, map2y, cv::INTER_LINEAR);

    leftImage = rectifiedLeft.clone();
    rightImage = rectifiedRight.clone();

    // return std::make_pair(rectifiedLeft, rectifiedRight);
}
