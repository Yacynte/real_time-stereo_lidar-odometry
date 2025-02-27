// StereoCamera.h
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp> // ORB and feature detection
#include <opencv2/calib3d.hpp>
#include <cassert> // Include this header
#include <opencv2/core/cvstd.hpp>

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <filesystem> // For directory and file handling

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
};

class VisualOdometry {
    public:
        // Method to rectify images
        std::pair<cv::Mat, cv::Mat> RectifyImage(const cv::Mat& leftImage, const cv::Mat& rightImage);

        // Method to compute stereo odometry
        cv::Mat StereoOdometry(cv::Mat leftImage_pre, cv::Mat leftImage_cur,
                        cv::Mat rightImage_pre, cv::Mat rightImage_cur); //, cv::Mat init_R, cv::Mat init_T);

        // Motion estimation from two sets of 2D points and depth map.
        bool motionEstimation(const std::vector<cv::Point2f>& image1_points, const std::vector<cv::Point2f>& image2_points,
                            const cv::Mat& depth, float max_depth = 500.0f);

        void updatePose(std::vector<cv::Mat>& T_prev, cv::Mat& T_rel, int id);

        // const std::vector<cv::Point2f> &contours, cv::Mat &contour_3d,

        // void feature_matching(const cv::Mat &left_prev, const cv::Mat &left_cur, 
        //                     std::vector<cv::Point2f> &pts_prev_L, std::vector<cv::Point2f> &pts_cur_L); 
        // void feature_matching(const cv::Mat &left_prev, const cv::Mat &left_cur, 
        //                     std::vector<cv::Point2f> &pts_prev_L, std::vector<cv::Point2f> &pts_cur_L) {
    private:
        // Step 2: Define camera parameters
        cv::Mat K1 = (cv::Mat_<double>(3, 3) << 320, 0, 240, 0, 320, 240, 0, 0, 1); // Left camera intrinsic
        cv::Mat K2 = (cv::Mat_<double>(3, 3) << 320, 0, 240, 0, 320, 240, 0, 0, 1); // Right camera intrinsic
        cv::Mat D1 = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0); // Left camera distortion
        cv::Mat D2 = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0); // Right camera distortion
        cv::Mat R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1.0000); // Rotation between cameras
        cv::Mat T = (cv::Mat_<double>(3, 1) << 0.085, 0, 0); // Translation between cameras
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        // cv::Mat disparity;
        // cv::Mat depth_map;
        cv::Mat rvec;                           // Sotre Rotation vector
        cv::Mat rotation_matrix;                // Sotre Rotation matrix 
        cv::Mat translation_vector;
        // std::vector<cv::Mat> totalRotation;
        // std::vector<cv::Mat> totalTranslation;
        // cv::Mat depth; 
        // Rectified images
        cv::Mat leftImageRec_pre, leftImageRec_cur;
        cv::Mat rightImageRec_pre, rightImageRec_cur;
        // double baseline_;      // Baseline distance between the stereo cameras
        // double focal_length_;  // Focal length of the left camera
        // Reconstruct 3D points from 2D points using depth information.
        void reconstruct3D(const std::vector<cv::Point2f>& image_points, const cv::Mat& depth,
                        std::vector<cv::Point3f>& points_3D, std::vector<size_t>& outliers, float max_depth);

        //Compute mean of 3D points.
        cv::Point3f computeMean3D(const std::vector<cv::Point3f>& points);

        // Compute Disparity
        cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right);

        // Compute depth map
        cv::Mat computeDepth(const cv::Mat& left, const cv::Mat& right); 

        void feature_matching(const cv::Mat& left_prev, const cv::Mat& left_cur, 
                            std::vector<cv::Point2f>& pts_prev_L, std::vector<cv::Point2f>& pts_cur_L);
    };


class ImageLoader {
    public:
        // Constructor: takes the folder path and file extension filter
        ImageLoader(const std::string& folderPath, const std::string& extension = ".png")
            : folderPath(folderPath), fileExtension(extension) {}
        // Load all images from the folder into a vector of cv::Mat
        bool loadImages(std::vector<cv::Mat>& images);
    private:
        std::string folderPath;      // Path to the folder containing images
        std::string fileExtension;   // File extension filter (e.g., ".png", ".jpg")
};