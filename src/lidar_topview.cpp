#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dataStructures.h"
#include "lidarData.hpp"

using namespace std;

// Function to visualize Lidar points
void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, cv::Mat &outputImg)
{
    // Create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Plot Lidar points
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = it->x; // world position in m with x facing forward from sensor
        float yw = it->y; // world position in m with y facing left from sensor

        // Top-view coordinates
        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

        // Color based on distance
        float val = it->x;
        float maxVal = worldSize.height;
        int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
        int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
        cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1);
    }

    // Return the image
    outputImg = topviewImg.clone();
}

int main()
{
    // Data location
    string dataPath = "../";
    string imgBasePath = dataPath + "images/";
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";
    int imgFillWidth = 4;  // no. of digits which make up the file index

    // Process specific frames that might show anomalies
    vector<int> frameIndices = {1, 4, 7, 12, 15, 18}; // Example frames to process

    for (auto imgIndex : frameIndices)
    {
        // Load Lidar points
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgIndex;
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        
        cout << "Processing frame " << imgIndex << " from " << lidarFullFilename << endl;
        
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);
        
        // Remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
        
        // Generate and save top-view image
        cv::Mat topviewImg;
        showLidarTopview(lidarPoints, cv::Size(25.0, 50.0), cv::Size(1000, 2000), topviewImg);
        
        // Add frame number to the image
        string frameText = "Frame: " + to_string(imgIndex);
        cv::putText(topviewImg, frameText, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
        
        // Save the image
        string outputFilename = "../images/FP5_lidar_frame" + to_string(imgIndex) + ".png";
        cv::imwrite(outputFilename, topviewImg);
        cout << "Saved " << outputFilename << endl;
    }
    
    cout << "Lidar top-view images generated successfully!" << endl;
    return 0;
}
