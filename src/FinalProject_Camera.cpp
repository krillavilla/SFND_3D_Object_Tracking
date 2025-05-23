#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>

#include <opencv2/opencv.hpp>

// project headers
#include "dataStructures.h"       // DataFrame, BoundingBox, LidarPoint
#include "camFusion.hpp"          // computeTTCCamera, matchBoundingBoxes, clusterLidarWithROI, associateKeypointsWithROI
#include "lidarData.hpp"          // computeTTCLidar, loadLidarFromFile
#include "matching2D.hpp"         // createDetector, createDescriptor, matchDescriptors
#include "objectDetection2D.hpp"   // detectObjects

// parse comma-separated list into vector<string>
static std::vector<std::string> parseCommaSeparatedList(const std::string& input)
{
    std::vector<std::string> tokens;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty())
            tokens.push_back(item);
    }
    return tokens;
}

// Function to display usage information
static void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --detectors <list>     Comma-separated list of detectors (default: FAST,BRISK,ORB,AKAZE,SIFT)" << std::endl;
    std::cout << "  --descriptors <list>   Comma-separated list of descriptors (default: BRIEF,FREAK,ORB,AKAZE,SIFT)" << std::endl;
    std::cout << "  --batch <file>         Process a batch of frames from a file" << std::endl;
    std::cout << "  --camera_sweep         Run the full detector/descriptor analysis" << std::endl;
    std::cout << "  --log <file>           Specify the output CSV file (default: results_full.csv)" << std::endl;
    std::cout << "  --help                 Display this help message" << std::endl;
}

/**
 * Process a single frame: load image & lidar, detect & match features, compute TTC.
 * Returns pair<ttcCamera, ttcLidar>.
 */
static std::pair<double,double> processFrame(
    const std::string& detectorType,
    const std::string& descriptorType,
    int frameId,
    std::vector<DataFrame>& dataBuffer)
{
    // --- 1) Camera calibration matrices (fill with your own values) ---
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type);
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type);
    cv::Mat RT(4,4,cv::DataType<double>::type);

    // Initialize calibration matrices with KITTI values
    P_rect_00.at<double>(0,0) = 7.215377e+02;
    P_rect_00.at<double>(0,1) = 0.000000e+00;
    P_rect_00.at<double>(0,2) = 6.095593e+02;
    P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00;
    P_rect_00.at<double>(1,1) = 7.215377e+02;
    P_rect_00.at<double>(1,2) = 1.728540e+02;
    P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00;
    P_rect_00.at<double>(2,1) = 0.000000e+00;
    P_rect_00.at<double>(2,2) = 1.000000e+00;
    P_rect_00.at<double>(2,3) = 0.000000e+00;

    R_rect_00.at<double>(0,0) = 1.0;
    R_rect_00.at<double>(0,1) = 0.0;
    R_rect_00.at<double>(0,2) = 0.0;
    R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = 0.0;
    R_rect_00.at<double>(1,1) = 1.0;
    R_rect_00.at<double>(1,2) = 0.0;
    R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 0.0;
    R_rect_00.at<double>(2,1) = 0.0;
    R_rect_00.at<double>(2,2) = 1.0;
    R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0.0;
    R_rect_00.at<double>(3,1) = 0.0;
    R_rect_00.at<double>(3,2) = 0.0;
    R_rect_00.at<double>(3,3) = 1.0;

    RT.at<double>(0,0) = 7.533745e-03;
    RT.at<double>(0,1) = -9.999714e-01;
    RT.at<double>(0,2) = -6.166020e-04;
    RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02;
    RT.at<double>(1,1) = 7.280733e-04;
    RT.at<double>(1,2) = -9.998902e-01;
    RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01;
    RT.at<double>(2,1) = 7.523790e-03;
    RT.at<double>(2,2) = 1.480755e-02;
    RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0;
    RT.at<double>(3,1) = 0.0;
    RT.at<double>(3,2) = 0.0;
    RT.at<double>(3,3) = 1.0;

    // --- 2) Load image ---
    std::ostringstream imgNumber;
    imgNumber << std::setw(6) << std::setfill('0') << frameId;
    std::string imgFullFilename = "../images/KITTI/2011_09_26/image_02/data/" + imgNumber.str() + ".png";
    cv::Mat img = cv::imread(imgFullFilename);

    if (img.empty()) {
        std::cerr << "Error: Could not read image " << imgFullFilename << std::endl;
        return {-1.0, -1.0};
    }

    DataFrame frame;
    frame.cameraImg = img;
    dataBuffer.push_back(frame);

    // --- 3) Load and crop Lidar points ---
    std::string lidarFullFilename = "../images/KITTI/2011_09_26/velodyne_points/data/" + imgNumber.str() + ".bin";
    std::vector<LidarPoint> lidarPoints;
    loadLidarFromFile(lidarPoints, lidarFullFilename);

    // Remove Lidar points based on distance properties (focus on ego lane)
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1;
    cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

    // Store Lidar points in the frame
    frame.lidarPoints = lidarPoints;

    // --- 4) Create a predefined bounding box instead of using object detection ---
    // This is a workaround for the YOLO weights issue
    BoundingBox box;
    box.roi = cv::Rect(535, 180, 180, 150); // Approximate position of the preceding vehicle
    box.boxID = 0;
    box.trackID = 0;
    box.classID = 1; // Car class
    box.confidence = 1.0;
    frame.boundingBoxes.push_back(box);

    std::cout << "Using predefined bounding box for frame " << frameId << std::endl;

    // --- 5) Cluster Lidar with ROI ---
    float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
    clusterLidarWithROI(dataBuffer.back().boundingBoxes, frame.lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

    // --- 6) Keypoint detection and description ---
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Handle special detector types (SHITOMASI, HARRIS) separately
    if (detectorType.compare("SHITOMASI") == 0) {
        // Convert to grayscale
        cv::Mat imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        detKeypointsShiTomasi(keypoints, imgGray, false);
    } else if (detectorType.compare("HARRIS") == 0) {
        // Convert to grayscale
        cv::Mat imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        detKeypointsHarris(keypoints, imgGray, false);
    } else {
        // Use the createDetector function for other detector types
        cv::Ptr<cv::Feature2D> detector = createDetector(detectorType);
        if (detector) {
            detector->detect(img, keypoints);
        } else {
            std::cerr << "Error: Failed to create detector for type " << detectorType << std::endl;
            return {-1.0, -1.0};
        }
    }

    // Extract descriptors
    cv::Ptr<cv::DescriptorExtractor> extractor = createDescriptor(descriptorType);
    if (extractor) {
        extractor->compute(img, keypoints, descriptors);
    } else {
        std::cerr << "Error: Failed to create descriptor extractor for type " << descriptorType << std::endl;
        return {-1.0, -1.0};
    }

    frame.keypoints = keypoints;
    frame.descriptors = descriptors;

    // --- 7) Match descriptors with previous frame ---
    if (dataBuffer.size() > 1) {
        DataFrame& prevFrame = dataBuffer[dataBuffer.size()-2];

        // Determine descriptor category based on type
        std::string descriptorCategory;
        if (descriptorType.compare("SIFT") == 0) {
            descriptorCategory = "DES_HOG";
        } else {
            descriptorCategory = "DES_BINARY";
        }

        std::string matcherType = "MAT_BF";
        std::string selectorType = "SEL_NN";

        matchDescriptors(prevFrame.keypoints, frame.keypoints,
                         prevFrame.descriptors, frame.descriptors,
                         frame.kptMatches, descriptorCategory, matcherType, selectorType);

        // --- 8) Match bounding boxes between frames ---
        std::map<int, int> bbBestMatches;
        matchBoundingBoxes(frame.kptMatches, bbBestMatches, prevFrame, frame);

        // Store matches in current data frame
        frame.bbMatches = bbBestMatches;

        // --- 9) Compute TTC for Lidar and Camera ---
        double ttcLidar;
        computeTTCLidar(
            prevFrame.lidarPoints, frame.lidarPoints,
            0.1, // frameRate
            ttcLidar);

        double ttcCamera;
        computeTTCCamera(
            prevFrame.keypoints, frame.keypoints,
            frame.kptMatches,
            0.1, // frameRate
            ttcCamera);

        return {ttcCamera, ttcLidar};
    }

    // Not enough frames yet
    return {-1.0, -1.0};
}

int main(int argc, char* argv[])
{
    // Default parameters
    std::string detectorsList = "FAST,BRISK,ORB,AKAZE,SIFT";
    std::string descriptorsList = "ORB,BRISK,AKAZE,SIFT"; // Removed BRIEF and FREAK as they require xfeatures2d
    std::string batchFile = "";
    std::string logFile = "results_full.csv";
    bool cameraSweep = false;

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--detectors") == 0 && i + 1 < argc) {
            detectorsList = argv[++i];
        } else if (strcmp(argv[i], "--descriptors") == 0 && i + 1 < argc) {
            descriptorsList = argv[++i];
        } else if (strcmp(argv[i], "--batch") == 0 && i + 1 < argc) {
            batchFile = argv[++i];
        } else if (strcmp(argv[i], "--camera_sweep") == 0) {
            cameraSweep = true;
        } else if (strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            logFile = argv[++i];
        } else {
            std::cerr << "Unknown option: " << argv[i] << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // Parse detector and descriptor lists
    auto detectors = parseCommaSeparatedList(detectorsList);
    auto descriptors = parseCommaSeparatedList(descriptorsList);

    // Open results CSV
    std::ofstream resultsFile(logFile);
    if (!resultsFile.is_open()) {
        std::cerr << "Error: Could not open log file " << logFile << std::endl;
        return 1;
    }

    // Write CSV header
    resultsFile << "detector,descriptor,frame,ttcCamera,ttcLidar\n";

    // Process frames based on command-line options
    if (cameraSweep) {
        // Sweep all valid detector/descriptor combinations
        for (const auto& det : detectors) {
            for (const auto& desc : descriptors) {
                // Skip invalid combinations
                if (!isValidDetectorDescriptorCombination(det, desc)) {
                    std::cout << "Skipping invalid combination: " << det << "+" << desc << std::endl;
                    continue;
                }

                std::cout << "Running: " << det << "+" << desc << std::endl;
                std::vector<DataFrame> dataBuffer;

                // Process frames 0..18 (or from batch file if specified)
                std::vector<int> frameIds;
                if (!batchFile.empty()) {
                    // Read frame IDs from batch file
                    std::ifstream batchStream(batchFile);
                    if (!batchStream.is_open()) {
                        std::cerr << "Error: Could not open batch file " << batchFile << std::endl;
                        return 1;
                    }

                    int frameId;
                    while (batchStream >> frameId) {
                        frameIds.push_back(frameId);
                    }
                } else {
                    // Use default frame range 0..18
                    for (int i = 0; i <= 18; ++i) {
                        frameIds.push_back(i);
                    }
                }

                // Process each frame
                for (int frameId : frameIds) {
                    auto [ttcCam, ttcLid] = processFrame(det, desc, frameId, dataBuffer);

                    // Write results to CSV file, even if camera TTC calculation fails
                    resultsFile << det << "," << desc << "," << frameId << ",";

                    if (ttcCam > 0) {
                        resultsFile << ttcCam;
                    } else {
                        resultsFile << "N/A";
                    }

                    resultsFile << ",";

                    if (ttcLid > 0) {
                        resultsFile << ttcLid;
                    } else {
                        resultsFile << "N/A";
                    }

                    resultsFile << "\n";

                    // Keep only last two frames
                    if (dataBuffer.size() > 2)
                        dataBuffer.erase(dataBuffer.begin());
                }
            }
        }
    } else {
        // Process only the first detector/descriptor pair
        if (detectors.empty() || descriptors.empty()) {
            std::cerr << "Error: No detectors or descriptors specified" << std::endl;
            return 1;
        }

        const auto& det = detectors[0];
        const auto& desc = descriptors[0];

        if (!isValidDetectorDescriptorCombination(det, desc)) {
            std::cerr << "Error: Invalid detector/descriptor combination: " << det << "+" << desc << std::endl;
            return 1;
        }

        std::cout << "Running: " << det << "+" << desc << std::endl;
        std::vector<DataFrame> dataBuffer;

        // Process frames 0..18 (or from batch file if specified)
        std::vector<int> frameIds;
        if (!batchFile.empty()) {
            // Read frame IDs from batch file
            std::ifstream batchStream(batchFile);
            if (!batchStream.is_open()) {
                std::cerr << "Error: Could not open batch file " << batchFile << std::endl;
                return 1;
            }

            int frameId;
            while (batchStream >> frameId) {
                frameIds.push_back(frameId);
            }
        } else {
            // Use default frame range 0..18
            for (int i = 0; i <= 18; ++i) {
                frameIds.push_back(i);
            }
        }

        // Process each frame
        for (int frameId : frameIds) {
            auto [ttcCam, ttcLid] = processFrame(det, desc, frameId, dataBuffer);

            // Write results to CSV file, even if camera TTC calculation fails
            resultsFile << det << "," << desc << "," << frameId << ",";

            if (ttcCam > 0) {
                resultsFile << ttcCam;
            } else {
                resultsFile << "N/A";
            }

            resultsFile << ",";

            if (ttcLid > 0) {
                resultsFile << ttcLid;
            } else {
                resultsFile << "N/A";
            }

            resultsFile << "\n";

            // Keep only last two frames
            if (dataBuffer.size() > 2)
                dataBuffer.erase(dataBuffer.begin());
        }
    }

    resultsFile.close();
    std::cout << "Processing complete. Results saved to " << logFile << std::endl;
    return 0;
}
