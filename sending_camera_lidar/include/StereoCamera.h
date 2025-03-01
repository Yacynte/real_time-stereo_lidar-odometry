// StereoCamera.h
#pragma once

#include <iostream>

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>



// #include <opencv2/core/parallel/parallel.hpp>
class StereoCamera {
    public:
        /**
         * Constructor initializes both left and right cameras using their IDs.
         * @param leftCamID - ID of the left camera
         * @param rightCamID - ID of the right camera
         */
        StereoCamera(int leftCamID, int rightCamID);

        /**
         * Captures a stereo pair of images (left and right).
         * @param leftFrame - Reference to store left image
         * @param rightFrame - Reference to store right image
         * @return true if frames were captured successfully, false otherwise
         */
        bool captureFrames(cv::Mat& leftFrame, cv::Mat& rightFrame);

        /**
         * Cleans up and releases the camera resources.
         */
        ~StereoCamera();
    private:
        cv::VideoCapture leftCam;   // Camera stream for the left view
        cv::VideoCapture rightCam;  // Camera stream for the right view
        int leftCamIDp; int rightCamIDp;

        /**
         * Checks if the cameras have been opened successfully.
         * @return true if both cameras are initialized properly, false otherwise
         */
        bool checkCameras();
        void splitStereoImage(cv::Mat& leftFrame, cv::Mat& rightFrame);

        cv::Mat K1 = (cv::Mat_<double>(3, 3) << 320, 0, 240, 0, 320, 240, 0, 0, 1); // Left camera intrinsic
        cv::Mat K2 = (cv::Mat_<double>(3, 3) << 320, 0, 240, 0, 320, 240, 0, 0, 1); // Right camera intrinsic
        cv::Mat D1 = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0); // Left camera distortion
        cv::Mat D2 = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0); // Right camera distortion
        cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1.0000); // Rotation between cameras
        cv::Mat T = (cv::Mat_<double>(3, 1) << 0.085, 0, 0); // Translation between cameras

        // Method to rectify images
        void RectifyImage(cv::Mat& leftImage, cv::Mat& rightImage);

        
};
