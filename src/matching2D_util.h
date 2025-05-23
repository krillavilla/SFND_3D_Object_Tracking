// Add these includes at the top
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

// Add these function declarations before main()
void processFrameWithDetectorDescriptor(
    const std::string& detectorType, 
    const std::string& descriptorType,
    std::vector<DataFrame>& dataBuffer,
    const std::string& imgBasePath, 
    const std::string& imgPrefix,
    const std::string& imgFileType,
    int imgStartIndex, int imgEndIndex, int imgStepWidth, int imgFillWidth,
    bool bVis, double& ttcCamera, double& ttcLidar,
    std::ofstream& resultsFile, int frameId);

std::vector<std::string> parseCommaSeparatedList(const std::string& input);

// In main(), add command-line argument parsing
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    
    // Parse command-line arguments
    bool batchMode = false;
    bool cameraSweep = false;
    std::string framesFile;
    std::string logFile = "results.csv";
    std::string detectorsList = "SHITOMASI,HARRIS,FAST,BRISK,ORB,AKAZE,SIFT";
    std::string descriptorsList = "BRISK,BRIEF,ORB,FREAK,AKAZE,SIFT";
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--batch") {
            batchMode = true;
            if (i + 1 < argc) {
                framesFile = argv[++i];
            }
        } else if (arg == "--log") {
            if (i + 1 < argc) {
                logFile = argv[++i];
            }
        } else if (arg == "--camera_sweep") {
            cameraSweep = true;
        } else if (arg == "--detectors") {
            if (i + 1 < argc) {
                detectorsList = argv[++i];
            }
        } else if (arg == "--descriptors") {
            if (i + 1 < argc) {
                descriptorsList = argv[++i];
            }
        }
    }
    
    // Parse detector and descriptor lists
    std::vector<std::string> detectors = parseCommaSeparatedList(detectorsList);
    std::vector<std::string> descriptors = parseCommaSeparatedList(descriptorsList);
    
    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

    // Create output file for results
    std::ofstream resultsFile;
    resultsFile.open(logFile);
    
    // Write header for results file
    if (cameraSweep) {
        resultsFile << "detector,descriptor,frameId,ttc_camera,ttc_lidar" << std::endl;
    } else {
        resultsFile << "frameId,ttc_lidar,ttc_camera" << std::endl;
    }

    if (cameraSweep) {
        // Run camera TTC sweep with all detector/descriptor combinations
        for (const auto& detectorType : detectors) {
            for (const auto& descriptorType : descriptors) {
                // Skip invalid combinations
                if ((detectorType == "SIFT" && descriptorType == "ORB") ||
                    (descriptorType == "AKAZE" && detectorType != "AKAZE") ||
                    (descriptorType == "SIFT" && (detectorType == "ORB" || detectorType == "FAST"))) {
                    std::cout << "Skipping incompatible combination: " << detectorType << " + " << descriptorType << std::endl;
                    continue;
                }
                
                std::cout << "Processing with detector=" << detectorType << ", descriptor=" << descriptorType << std::endl;
                
                // Reset data buffer for each combination
                std::vector<DataFrame> dataBuffer;
                
                // Process all frames with current detector/descriptor combination
                for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth) {
                    // Compute frame ID
                    int frameId = imgStartIndex + imgIndex;
                    
                    // Process frame
                    double ttcCamera = 0.0, ttcLidar = 0.0;
                    processFrameWithDetectorDescriptor(
                        detectorType, descriptorType, dataBuffer,
                        imgBasePath, imgPrefix, imgFileType,
                        imgStartIndex, imgEndIndex, imgStepWidth, imgFillWidth,
                        false, ttcCamera, ttcLidar,
                        resultsFile, frameId);
                }
            }
        }
    } else {
        // Original processing loop with default detector/descriptor
        // ... [original code here]
    }

    return 0;
}

// Helper function to parse comma-separated list
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

// Function to process a frame with specific detector/descriptor
void processFrameWithDetectorDescriptor(
    const std::string& detectorType, 
    const std::string& descriptorType,
    std::vector<DataFrame>& dataBuffer,
    const std::string& imgBasePath, 
    const std::string& imgPrefix,
    const std::string& imgFileType,
    int imgStartIndex, int imgEndIndex, int imgStepWidth, int imgFillWidth,
    bool bVis, double& ttcCamera, double& ttcLidar,
    std::ofstream& resultsFile, int frameId) {
    
    // Assemble filenames for current index
    std::ostringstream imgNumber;
    imgNumber << std::setfill('0') << std::setw(imgFillWidth) << frameId;
    std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
    
    // Load image from file
    cv::Mat img = cv::imread(imgFullFilename);
    
    // Push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = img;
    
    // ... [rest of the processing code, similar to the original loop]
    
    // Write results to file
    resultsFile << detectorType << "," << descriptorType << "," << frameId << "," 
                << ttcCamera << "," << ttcLidar << std::endl;
}