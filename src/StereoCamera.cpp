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

    splitStereoImage(leftFrame, rightFrame);
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

void VisualOdometry::reconstruct3D(const std::vector<cv::Point2f>& image_points, const cv::Mat& depth,
                                   std::vector<cv::Point3f>& points_3D, std::vector<size_t>& outliers, float max_depth) {
    points_3D.clear();
    outliers.clear();

    

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

    cv::Point3f mean = std::accumulate(points.begin(), points.end(), cv::Point3f(0, 0, 0));
    mean *= (1.0f / points.size());
    return mean;
}

bool VisualOdometry::motionEstimation(const std::vector<cv::Point2f>& image1_points, const std::vector<cv::Point2f>& image2_points,
    const cv::Mat& depth, float max_depth) {
    if (image1_points.size() != image2_points.size()) {
        std::cerr << "Error: Point sets must have the same size." << std::endl;
        return false;
    }

    // Step 1: Reconstruct 3D points
    std::vector<cv::Point3f> points_3D;
    std::vector<size_t> outliers;
    reconstruct3D(image1_points, depth, points_3D, outliers, max_depth);

    // Step 2: Use an unordered_set for fast outlier lookup
    std::unordered_set<size_t> outlier_set(outliers.begin(), outliers.end());

    // Filter points, reserving space to avoid multiple reallocations
    std::vector<cv::Point2f> filtered_image1_points, filtered_image2_points;
    std::vector<cv::Point3f> filtered_points_3D;
    filtered_image1_points.reserve(image1_points.size());
    filtered_image2_points.reserve(image2_points.size());
    filtered_points_3D.reserve(points_3D.size());

    for (size_t i = 0; i < image1_points.size(); ++i) {
        if (outlier_set.find(i) == outlier_set.end()) {
            filtered_image1_points.push_back(image1_points[i]);
            filtered_image2_points.push_back(image2_points[i]);
            filtered_points_3D.push_back(points_3D[i]);
        }
    }

    

    // Step 3: Solve PnP with RANSAC
    cv::Mat points_3D_mat(filtered_points_3D), filtered_image2_points_mat(filtered_image2_points);
    cv::Mat(points_3D).convertTo(points_3D_mat, CV_32F);
    cv::Mat(filtered_image2_points).convertTo(filtered_image2_points_mat, CV_32F);

    // std::vector<int> inliers;
    bool success = cv::solvePnPRansac(points_3D_mat, filtered_image2_points_mat, K1_float, D1_float,
        rvec, translation_vector, false, 100, 8.0, 0.99, cv::noArray());

    if (!success) {
        std::cerr << "Error: solvePnPRansac failed." << std::endl;
        return false;
    }

    // Convert rotation vector to matrix
    cv::Rodrigues(rvec, rotation_matrix);

    // Optionally, filter inliers after PnP
    // std::vector<cv::Point2f> inlier_image1_points, inlier_image2_points;
    // for (size_t i = 0; i < inliers.size(); ++i) {
    //     inlier_image1_points.push_back(image1_points[inliers[i]]);
    //     inlier_image2_points.push_back(image2_points[inliers[i]]);
    // }

    return true;
}

cv::Mat VisualOdometry::computeDisparity(const cv::Mat& left, const cv::Mat& right) {
    // Resize images for faster computation (optional, depending on accuracy needs)
    // cv::Mat left_resized, right_resized;
    // cv::resize(left, left_resized, cv::Size(640, 480));
    // cv::resize(right, right_resized, cv::Size(640, 480));

    // Create StereoSGBM object
    auto stereo = cv::StereoSGBM::create(
        0,            // Min disparity
        4 * 16,       // Reduced number of disparities for faster computation
        7,            // Smaller block size for faster computation
        8 * 7 * 7,    // P1 with block size of 5
        32 * 7 * 7,   // P2 with block size of 5
        31,           // Pre-filter cap
        10,           // Uniqueness ratio
        0,            // Speckle window size
        0,            // Speckle range
        cv::StereoSGBM::MODE_SGBM // Mode
    );

    // Compute disparity
    cv::Mat disparity(left.size(), CV_16S);
    stereo->compute(left, right, disparity);

    // Normalize disparity for visualization (optional)
    cv::Mat disparity_normalized;
    cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    return disparity;
}
cv::Mat VisualOdometry::computeDepth(const cv::Mat& left, const cv::Mat& right) {
    auto disparity = computeDisparity(left, right);  // Assuming this is a pre-defined function

    double focal_length_ = K1.at<double>(0, 0);
    double baseline_ = T.at<double>(0);

    cv::Mat depth(disparity.size(), CV_32F);

    // Define a parallel loop for processing each pixel
    cv::parallel_for_(cv::Range(0, disparity.rows), [&](const cv::Range& r) {
        for (int y = r.start; y < r.end; ++y) {
            for (int x = 0; x < disparity.cols; ++x) {
                float d = static_cast<float>(disparity.at<short>(y, x)) / 16.0;  // SGBM divides disparity by 16
                if (d > 0) { // Avoid division by zero
                    depth.at<float>(y, x) = (focal_length_ * baseline_) / d;
                } else {
                    depth.at<float>(y, x) = 0.0f; // Invalid depth
                }
            }
        }
    });

    return depth;
}


// Feature matching function
void VisualOdometry::feature_matching(const cv::Mat& left_prev, const cv::Mat& left_cur, 
    std::vector<cv::Point2f>& pts_prev_L, std::vector<cv::Point2f>& pts_cur_L) {
    const int MAX_FEATURES = 500; // Reduce the number of keypoints for faster matching
    cv::Ptr<cv::ORB> orb = cv::ORB::create(MAX_FEATURES);

    std::vector<cv::KeyPoint> keypoints_prev, keypoints_cur;
    cv::Mat descriptors_prev, descriptors_cur;

    // Run feature detection and description in parallel
    std::thread t1([&]() { orb->detectAndCompute(left_prev, cv::noArray(), keypoints_prev, descriptors_prev); });
    std::thread t2([&]() { orb->detectAndCompute(left_cur, cv::noArray(), keypoints_cur, descriptors_cur); });

    t1.join();
    t2.join();

    cv::Mat descriptors_prev_float, descriptors_cur_float;
    descriptors_prev.convertTo(descriptors_prev_float, CV_32F); // Convert to CV_32F
    descriptors_cur.convertTo(descriptors_cur_float, CV_32F);   // Convert to CV_32F

    // Step 2: Use FLANN-based matching for speed (use Hamming distance for ORB)
    cv::FlannBasedMatcher flann_matcher;
    std::vector<std::vector<cv::DMatch>> knn_matches;

    // KNN match descriptors with 2 nearest neighbors (for Lowe's ratio test)
    flann_matcher.knnMatch(descriptors_prev_float, descriptors_cur_float, knn_matches, 2);

    // Step 3: Apply Lowe's ratio test (filter matches)
    const float RATIO_THRESHOLD = 0.75f;
    std::vector<cv::DMatch> good_matches;

    // Parallelize match processing to optimize for loop
    #pragma omp parallel for
    for (int i = 0; i < knn_matches.size(); ++i) {
    if (knn_matches[i][0].distance < RATIO_THRESHOLD * knn_matches[i][1].distance) {
        good_matches.push_back(knn_matches[i][0]); // Only keep good matches
        }
    }

    // Step 4: Extract matched keypoints
    pts_prev_L.clear();
    pts_cur_L.clear();

    // Parallelize extraction of matched keypoints
    #pragma omp parallel for
    for (int i = 0; i < good_matches.size(); ++i) {
    pts_prev_L.push_back(keypoints_prev[good_matches[i].queryIdx].pt);
    pts_cur_L.push_back(keypoints_cur[good_matches[i].trainIdx].pt);
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
    // auto rectifiedPre = RectifyImage(leftImage_pre, rightImage_pre);
    // // auto rectifiedCur = RectifyImage(leftImage_cur, rightImage_cur);

    // leftImageRec_pre = rectifiedPre.first;
    // rightImageRec_pre = rectifiedPre.second;
    // leftImageRec_cur = rectifiedCur.first;
    // rightImageRec_cur = rectifiedCur.second;

    //Compute depth map of previous Image pair
    // std::cout<<"Compute Depth"<<std::endl;
    auto depth_map = computeDepth(leftImage_pre, rightImage_pre);

    // Vectors to hold the matched points
    // std::cout<<"Init 2d points"<<std::endl;
    std::vector<cv::Point2f> pts_prev_L, pts_cur_L;

    // Call the feature matching function
    // std::cout<<"feature matching"<<std::endl;
    feature_matching(leftImage_pre, leftImage_cur, pts_prev_L, pts_cur_L);
    
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