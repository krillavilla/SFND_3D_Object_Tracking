// camera_sweep.cpp
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

// Function to parse comma-separated list
std::vector<std::string> parseCommaSeparatedList(const std::string& input) {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string item;

    while (std::getline(ss, item, ',')) {
        // Trim whitespace
        item.erase(0, item.find_first_not_of(" \t"));
        item.erase(item.find_last_not_of(" \t") + 1);

        if (!item.empty()) {
            result.push_back(item);
        }
    }

    return result;
}

// Function to check if detector/descriptor combination is valid
bool isValidCombination(const std::string& detectorType, const std::string& descriptorType) {
    // AKAZE descriptors only work with AKAZE keypoints
    if (descriptorType == "AKAZE" && detectorType != "AKAZE") {
        return false;
    }

    // ORB descriptors don't work well with SIFT keypoints
    if (descriptorType == "ORB" && detectorType == "SIFT") {
        return false;
    }

    // SIFT descriptors don't work well with FAST or ORB keypoints
    if (descriptorType == "SIFT" && (detectorType == "FAST" || detectorType == "ORB")) {
        return false;
    }

    return true;
}

// Function to run camera TTC with specific detector/descriptor
void runCameraTTC(
    const std::string& detectorType,
    const std::string& descriptorType,
    const std::string& imgBasePath,
    const std::string& imgPrefix,
    const std::string& imgFileType,
    const std::string& lidarPrefix,
    const std::string& lidarFileType,
    int imgStartIndex,
    int imgEndIndex,
    int imgStepWidth,
    int imgFillWidth,
    double frameRate,
    std::ofstream& resultsFile,
    cv::Mat& P_rect_00,
    cv::Mat& R_rect_00,
    cv::Mat& RT) {

    // Data buffer for current detector/descriptor combination
    std::vector<DataFrame> dataBuffer;

    // Process all frames
    for (int imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth) {
        // Current frame ID
        int frameId = imgStartIndex + imgIndex;

        // Assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << frameId;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // Load image from file
        cv::Mat img = cv::imread(imgFullFilename);

        // Push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);

        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

        // Detect keypoints
        std::vector<cv::KeyPoint> keypoints;

        if (detectorType == "SHITOMASI") {
            detKeypointsShiTomasi(keypoints, img, false);
        } else if (detectorType == "HARRIS") {
            detKeypointsHarris(keypoints, img, false);
        } else {
            detKeypointsModern(keypoints, img, detectorType, false);
        }

        // Add keypoints to current data frame
        (dataBuffer.end() - 1)->keypoints = keypoints;

        // Extract descriptors
        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        // Add descriptors to current data frame
        (dataBuffer.end() - 1)->descriptors = descriptors;

        // Load Lidar points
        std::string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // Add Lidar points to current data frame
        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

        // Only process TTC if we have at least two frames
        if (dataBuffer.size() > 1) {
            // Match keypoints between frames
            std::vector<cv::DMatch> matches;

            // Determine descriptor category based on type
            std::string descriptorCategory;
            if (descriptorType.compare("SIFT") == 0) {
                descriptorCategory = "DES_HOG";
            } else {
                descriptorCategory = "DES_BINARY";
            }

            std::string matcherType = "MAT_BF";
            std::string selectorType = "SEL_NN";

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                            (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                            matches, descriptorCategory, matcherType, selectorType);

            // Add matches to current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            // Compute TTC with Lidar
            double ttcLidar;
            computeTTCLidar((dataBuffer.end() - 2)->lidarPoints, (dataBuffer.end() - 1)->lidarPoints,
                           frameRate, ttcLidar);

            // Compute TTC with Camera
            double ttcCamera;
            computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                            (dataBuffer.end() - 1)->kptMatches, frameRate, ttcCamera);

            // Write results to file
            resultsFile << detectorType << "," << descriptorType << "," << frameId << ","
                       << ttcCamera << "," << ttcLidar << std::endl;
        } else {
            // First frame has no TTC
            resultsFile << detectorType << "," << descriptorType << "," << frameId << ",0,0" << std::endl;
        }

        // Limit data buffer size
        if (dataBuffer.size() > 2) {
            dataBuffer.erase(dataBuffer.begin());
        }
    }
}