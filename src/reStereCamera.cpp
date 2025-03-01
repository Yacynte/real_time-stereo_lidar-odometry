// StereoCamera.cpp
#include "StereoCamera.h"

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

std::pair<cv::Mat, cv::Mat> VisualOdometry::RectifyImage(const cv::Mat& leftImage, const cv::Mat& rightImage) {
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

    return std::make_pair(rectifiedLeft, rectifiedRight);
}

void VisualOdometry::reconstruct3D(const std::vector<cv::Point2f>& image_points, const cv::Mat& depth,
                                   std::vector<cv::Point3f>& points_3D, std::vector<size_t>& outliers, float max_depth) {
    points_3D.clear();
    outliers.clear();

    float fx = K1.at<double>(0, 0);
    float fy = K1.at<double>(1, 1);
    float cx = K1.at<double>(0, 2);
    float cy = K1.at<double>(1, 2);

    for (size_t i = 0; i < image_points.size(); ++i) {
        float u = image_points[i].x;
        float v = image_points[i].y;
        float z = depth.at<float>(static_cast<int>(v), static_cast<int>(u));

        // Ignore points with invalid depth
        if (z > max_depth) {
            outliers.push_back(i);
            continue;
        }

        float x = z * (u - cx) / fx;
        float y = z * (v - cy) / fy;
        points_3D.emplace_back(x, y, z);
    }
    // std::cout << "image_points: " << image_points.size() << " points 3d: " << points_3D.size() << " outliers : " << outliers.size()  <<std::endl;
}

cv::Point3f VisualOdometry::computeMean3D(const std::vector<cv::Point3f>& points) {
    if (points.empty()) return cv::Point3f(0, 0, 0);

    cv::Point3f mean(0, 0, 0);
    for (const auto &pt : points) {
        mean += pt;
    }
    mean *= (1.0f / points.size());
    return mean;
}

bool VisualOdometry::motionEstimation(const std::vector<cv::Point2f>& image1_points, const std::vector<cv::Point2f>& image2_points,
                                      const cv::Mat& depth, float max_depth) {
    // const std::vector<cv::Point2f> &contours, cv::Mat &contour_3d,
    if (image1_points.size() != image2_points.size()) {
        std::cerr << "Error: Point sets must have the same size." << std::endl;
        return false;
    }

    // Step 1: Reconstruct 3D points
    std::vector<cv::Point3f> points_3D;
    std::vector<size_t> outliers;
    // std::cout<<"Motion Estimation, reconstruct 3d"<<std::endl;
    reconstruct3D(image1_points, depth, points_3D, outliers, max_depth);

    // Remove outliers
    std::vector<cv::Point2f> filtered_image1_points, filtered_image2_points;
    for (size_t i = 0; i < image1_points.size(); ++i) {
        if (std::find(std::begin(outliers), std::end(outliers), i) == outliers.end()) {
            filtered_image1_points.push_back(image1_points[i]);
            filtered_image2_points.push_back(image2_points[i]);
            }
    }
    // std::cout<<"Motion Estimation, Outliers removed"<<std::endl;

    // Step 2: Solve PnP with RANSAC
    // std::cout << "Number of points before outlier regection: "<< points_3D.size() <<std::endl;
    // std::cout << "Number of points after outlier regection: "<< filtered_image2_points.size() <<std::endl;

    // Convert to float as OpenCV expects these inputs to be in CV_32F
    cv::Mat K1_float, D1_float, R_float, T_float, filtered_image2_points_, points_3D_ ;
    K1.convertTo(K1_float, CV_32F);
    D1.convertTo(D1_float, CV_32F);
    // R.convertTo(R_float, CV_32F);
    // T.convertTo(T_float, CV_32F);

    // Make sure your points are also of type CV_32F
    cv::Mat(points_3D).convertTo(points_3D_, CV_32F);
    cv::Mat(filtered_image2_points).convertTo(filtered_image2_points_, CV_32F);

    bool success = cv::solvePnPRansac(points_3D_, 
                                    filtered_image2_points_, 
                                    K1_float, D1_float, 
                                    rvec, translation_vector, false, 100, 8.0, 0.99, cv::noArray());
                
    // std::cout << "points_3D_: " << R_float << std::endl;
    if (!success) {
        std::cerr << "Error: solvePnPRansac failed." << std::endl;
        return false;
    }

    // Convert rotation vector to matrix
    cv::Rodrigues(rvec, rotation_matrix);



    // Step 3: Compute mean of contours in 3D if available
    // if (!contours.empty()) {
    //     std::vector<cv::Point3f> contours_3D;
    //     reconstruct3D(contours, depth, contours_3D, outliers, max_depth);
    //     cv::Point3f mean_3D = computeMean3D(contours_3D);

    //     // Convert to cv::Mat format
    //     contour_3d = (cv::Mat_<float>(1, 3) << mean_3D.x, mean_3D.y, mean_3D.z);
    // } else {
    //     contour_3d = cv::Mat::zeros(1, 3, CV_32F);
    // }

    return true;
}

cv::Mat VisualOdometry::computeDisparity(const cv::Mat& left, const cv::Mat& right) {
    // Create StereoSGBM object
    
    // cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();
    // stereo->setBlockSize(9);                    // Block size (odd number, typically 5 to 15)
    // stereo->setNumDisparities(4*16);            // Number of disparities (must be divisible by 16)
    // stereo->setPreFilterCap(31);
    // stereo->setBlockSize(9);
    // stereo->setP1(8 * 9 * 9);
    // stereo->setP2(32 * 9 * 9);
    // stereo->setMode(cv::StereoSGBM::MODE_SGBM);

    // Create StereoSGBM object and set parameters directly in the create function
    // auto stereo = cv::StereoSGBM::create();
    // cv::Ptr<cv::StereoSGBM> 
    // cv::Ptr<cv::StereoSGBM>
    // std::cout<< "Init Stereo" <<std::endl;
    auto stereo = cv::StereoSGBM::create(
        0,            // Min disparity
        4 * 16,       // Number of disparities (must be divisible by 16)
        9,            // Block size (odd number, typically 5 to 15)
        8 * 9 * 9,    // P1: The first parameter controlling the disparity smoothness
        32 * 9 * 9,   // P2: The second parameter controlling the disparity smoothness
        31,           // Pre-filter cap
        10,           // Uniqueness ratio
        0,            // Speckle window size
        0,            // Speckle range
        cv::StereoSGBM::MODE_SGBM // Mode
    );

    
    // Validate object creation
    // assert(!stereo.empty() && "StereoSGBM creation failed!");
    cv::Mat disparity(left.size(), CV_16S);
    // std::cout<< "Compute Disparity in Stereo" <<std::endl;
    stereo->compute(left, right, disparity); 
    // Normalize disparity for visualization (optional)
    // std::cout<< "Finish computing Disparity in Stereo" <<std::endl;
    cv::Mat disparity_normalized;
    cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    // std::cout<< "Finish Disparity in Stereo" <<std::endl;
    
    return disparity;
}

cv::Mat VisualOdometry::computeDepth(const cv::Mat& left, const cv::Mat& right){
    // const cv::Mat disparity = 
    // std::cout<< "Compute Disparity" <<std::endl;
    auto disparity = computeDisparity(left, right);
    // std::cout<< "Finish Computing Disparity" <<std::endl;
    double focal_length_ = K1.at<double>(0, 0);
    double baseline_ = T.at<double>(0);
    cv::Mat depth(disparity.size(), CV_32F);
    // std::cout<< "Depth" <<std::endl;
    for (int y = 0; y < disparity.rows; ++y) {
        for (int x = 0; x < disparity.cols; ++x) {
            float d = static_cast<float>(disparity.at<short>(y, x)) / 16.0; // SGBM divides disparity by 16
            if (d > 0) { // Avoid division by zero
                depth.at<float>(y, x) = (focal_length_ * baseline_) / d;
            } else {
                depth.at<float>(y, x) = 0.0f; // Invalid depth
            }
        }
    }
    return depth;
}

// Feature matching function
void VisualOdometry::feature_matching(const cv::Mat& left_prev, const cv::Mat& left_cur, 
                        std::vector<cv::Point2f>& pts_prev_L, std::vector<cv::Point2f>& pts_cur_L) {
    // Step 1: Detect ORB keypoints and compute descriptors for both images
    // cv::ORB orb; // Direct instantiation (deprecated in newer versions)
    // cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints_prev, keypoints_cur;
    cv::Mat descriptors_prev, descriptors_cur;

    // Detect keypoints and compute descriptors for the previous and current left images
    
    orb->detectAndCompute(left_prev, cv::noArray(), keypoints_prev, descriptors_prev);
    orb->detectAndCompute(left_cur, cv::noArray(), keypoints_cur, descriptors_cur);

    // Step 2: Match descriptors using Brute-Force Matcher (BFMatcher)
    cv::BFMatcher bf(cv::NORM_HAMMING, true);  // Use Hamming distance for ORB
    std::vector<cv::DMatch> matches;
    bf.match(descriptors_prev, descriptors_cur, matches);

    // Step 3: Extract the points from the matched keypoints
    pts_prev_L.clear();
    pts_cur_L.clear();
    
    for (size_t i = 0; i < matches.size(); i++) {
        // Get the keypoints from the matches
        pts_prev_L.push_back(keypoints_prev[matches[i].queryIdx].pt);
        pts_cur_L.push_back(keypoints_cur[matches[i].trainIdx].pt);
    }
}


cv::Mat VisualOdometry::StereoOdometry(cv::Mat leftImage_pre, cv::Mat leftImage_cur, cv::Mat rightImage_pre, cv::Mat rightImage_cur){
                                    // , cv::Mat init_R, cv::Mat init_T){

    if (leftImage_pre.empty() || leftImage_cur.empty() || rightImage_pre.empty() || rightImage_cur.empty()) {
        std::cerr << "One or all images are Empty!" << std::endl;
        cv::Mat rot = cv::Mat(3, 3, CV_32F, cv::Scalar(-1));
        cv::Mat trans =cv::Mat(3, 1, CV_32F, cv::Scalar(-1));
        return (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);
    }
     // Step 1: Rectify images
    // std::cout<<"rectify Image"<<std::endl;
    auto rectifiedPre = RectifyImage(leftImage_pre, rightImage_pre);
    auto rectifiedCur = RectifyImage(leftImage_cur, rightImage_cur);

    leftImageRec_pre = rectifiedPre.first;
    rightImageRec_pre = rectifiedPre.second;
    leftImageRec_cur = rectifiedCur.first;
    rightImageRec_cur = rectifiedCur.second;

    //Compute depth map of previous Image pair
    // std::cout<<"Compute Depth"<<std::endl;
    auto depth_map = computeDepth(leftImageRec_pre, rightImageRec_pre);

    // Vectors to hold the matched points
    // std::cout<<"Init 2d points"<<std::endl;
    std::vector<cv::Point2f> pts_prev_L, pts_cur_L;

    // Call the feature matching function
    // std::cout<<"feature matching"<<std::endl;
    feature_matching(leftImageRec_pre, leftImageRec_cur, pts_prev_L, pts_cur_L);
    
    // std::cout<<"Motion Estimation"<<std::endl;
    motionEstimation(pts_prev_L, pts_cur_L, depth_map);

    // totalTranslation.push_back(translation_vector);
    // totalRotation.push_back(rotation_matrix);
    // translation_vector += init_T;
    // rotation_matrix = init_R*rotation_matrix;
    
    // return std::make_pair(translation_vector, rotation_matrix);

     // Create a 4x4 transformation matrix and initialize with zeros
     cv::Mat transform = cv::Mat::eye(4, 4, CV_32F);

     // Set the top-left 3x3 portion to be the rotation matrix R
     rotation_matrix.copyTo(transform(cv::Range(0, 3), cv::Range(0, 3)));
 
     // Set the top-right 3x1 portion to be the translation vector t
     translation_vector.copyTo(transform(cv::Range(0, 3), cv::Range(3, 4)));
 
     // Set the last element (bottom-right corner) to 1 for homogeneous coordinates
    //  transform.at<double>(3, 3) = 1;

    return transform; 

}

void VisualOdometry::updatePose(std::vector<cv::Mat>& T_prev,  cv::Mat& T_rel, int id) {
    
    // Invert the transformation matrix (transf)
    // cv::Mat transf_inv = T_rel.inv();
    // cv::invert(T_rel, transf_inv);

    T_prev.push_back(T_prev.back() * T_rel);          // New rotation: R1 = R_rel * R0
    // T_prev.push_back(R_rel * T_prev.back() + T_rel);  // New translation: T1 = R_rel * T0 + T_rel
}

// Function to load all images from the folder into a vector
bool ImageLoader::loadImages(std::vector<cv::Mat>& images) {
    try {
        // Create a vector to store file paths
        std::vector<std::string> imagePaths;

        // Collect all image paths matching the extension
        for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
            if (entry.is_regular_file() && entry.path().extension() == fileExtension) {
                imagePaths.push_back(entry.path().string());
            }
        }

        // Sort the image paths alphabetically
        std::sort(imagePaths.begin(), imagePaths.end());

        // Load images in sorted order
        for (const auto& filePath : imagePaths) {
            cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);
            if (image.empty()) {
                std::cerr << "Failed to load image: " << filePath << std::endl;
                continue; // Skip invalid files
            }
            images.push_back(image);
            // Uncomment for debugging
            // std::cout << "Loaded: " << filePath << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
        return false;
    }
    return true;
}