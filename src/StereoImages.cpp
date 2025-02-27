#include <iostream>
#include <algorithm> 
#include <chrono>
#include "StereoCamera.h"
// #include <filesystem> // For creating folders (C++17)

void stereoOdom(std::vector<cv::Mat> leftImages, std::vector<cv::Mat> rightImages, VisualOdometry vo, 
                    std::vector<cv::Mat>& cumulativeTranslations, std::vector<cv::Mat>& cumulativeRotations, std::vector<cv::Mat>& timeTaken){

    auto startTime = std::chrono::steady_clock::now();
    auto start = std::chrono::steady_clock::now();
    int frameCounter = 0;
    cv::Mat leftFrame_pre, rightFrame_pre, leftFrame, rightFrame;
    // Initialize cumulative odometry
    cv::Mat cumulativeRotation = cv::Mat::eye(3, 3, CV_32F); // Identity matrix for rotation
    cv::Mat cumulativeTranslation = cv::Mat::zeros(3, 1, CV_32F); // Zero vector for translation

    // Vectors to store cumulative rotations and translations
    // std::vector<cv::Mat> cumulativeRotations;
    // std::vector<cv::Mat> cumulativeTranslations;

    for (int i = 0; i < leftImages.size(); i++) {
        leftFrame = leftImages[frameCounter];
        rightFrame = rightImages[frameCounter];
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();

        // Capture stereo frames
        if (!leftFrame.empty() && !rightFrame.empty()) {
            cv::imwrite("../data/left/left_" + std::to_string(frameCounter) + ".png", leftFrame);
            cv::imwrite("../data/right/right_" + std::to_string(frameCounter) + ".png", rightFrame);

            if (frameCounter > 0) {
                std::pair<cv::Mat, cv::Mat> motionPair;
                motionPair = vo.StereoOdometry(leftFrame_pre, leftFrame, rightFrame_pre, rightFrame);

                cv::Mat relativeRotation = motionPair.second; // Relative rotation
                cv::Mat relativeTranslation = motionPair.first; // Relative translation

                // Update cumulative odometry
                cumulativeRotation = cumulativeRotation * relativeRotation; // Update total rotation
                cumulativeTranslation += cumulativeRotation * relativeTranslation; // Update total translation

                // Store cumulative values in the vectors
                cumulativeRotations.push_back(cumulativeRotation.clone());
                cumulativeTranslations.push_back(cumulativeTranslation.clone());
            } else {
                // Push initial identity/zero for the first frame
                cumulativeRotations.push_back(cumulativeRotation.clone());
                cumulativeTranslations.push_back(cumulativeTranslation.clone());
            }
            
            // Update the pre-frame values here **after** processing
            leftFrame_pre = leftFrame.clone();
            rightFrame_pre = rightFrame.clone();

        } else {
            std::cerr << "Failed to capture stereo frames" << std::endl;
        }
        frameCounter++;
        startTime = std::chrono::steady_clock::now(); // Reset timer

        auto curTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - start).count();
        cv::Mat Ti = (cv::Mat_<double>(1, 1) << totalTime/1000);
        timeTaken.push_back(Ti);
        std::cout << ".";
        // Uncomment if you want to stop after a certain time
        // if (totalTime >= 4000) 
        //     break;
    }

    std::cout<< "\n";

}


int main(int argc, char* argv[]) {
    // Check if sufficient arguments are provided
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <left_folder_path> <right_folder_path> <extension>" << std::endl;
        return 1;
    }

    // Read arguments
    std::string leftFolderPath = argv[1];
    std::string rightFolderPath = argv[2];
    std::string extension = argv[3];

    // Create ImageLoader objects for left and right folders
    ImageLoader leftLoader(leftFolderPath, extension);
    ImageLoader rightLoader(rightFolderPath, extension);

    // Vectors to store the loaded images
    std::vector<cv::Mat> leftImages;
    std::vector<cv::Mat> rightImages;

    // Load images for the left and right folders
    if (leftLoader.loadImages(leftImages) && rightLoader.loadImages(rightImages)) {
        std::cout << "Successfully loaded " << leftImages.size() <<" left and "<< rightImages.size() << " right images." << std::endl;
    } else {
        std::cerr << "Error occurred while loading left images." << std::endl;
        return 0;
    }

    // Check if the number of left and right images match
    if (leftImages.size() != rightImages.size()) {
        std::cerr << "Warning: Mismatched number of left and right images." << std::endl;
        return 0;
    }


    VisualOdometry vo;

    std::vector<cv::Mat> cumulativeRotations;
    std::vector<cv::Mat> cumulativeTranslations;
    std::vector<cv::Mat> timeTaken;
    stereoOdom(leftImages, rightImages, vo, cumulativeTranslations, cumulativeRotations, timeTaken);
    
    cv::FileStorage fs("../transformations.json", cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

    fs << "TotalRotation" << "[";
    for (const auto& R : cumulativeRotations) {
        fs << R;
    }
    fs << "]";

    fs << "TotalTranslation" << "[";
    for (const auto& T : cumulativeTranslations) {
        fs << T; // Convert vector to Mat for saving
    }
    fs << "]";

    fs << "Time Taken" << "[";
    for (const auto& timetaken : timeTaken) {
        fs << timetaken; // Convert vector to Mat for saving
    }
    fs << "]";

    fs.release();

    // std::cout<<totalTranslation<<std::endl;
    // std::thread t1(camera_record, stereoCam, vo, &totalTranslation, &totalRotation);
    // std::thread t2(lidar_record, lidar);

    // t1.join();
    // t2.join();
    std::cout<< "Recording complete"<<"\n";
    return 0;
}
