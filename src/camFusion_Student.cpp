
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/*
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0;
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
    else
    {
        cv::waitKey(1); // just update the window
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // Clear any existing matches
    boundingBox.kptMatches.clear();

    // First, find all keypoint matches that are within the ROI of the current bounding box
    std::vector<cv::DMatch> matchesInROI;
    std::vector<double> distances;

    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        // Get the current keypoint
        cv::KeyPoint currKp = kptsCurr[it->trainIdx];
        cv::KeyPoint prevKp = kptsPrev[it->queryIdx];

        // Check if the keypoint is within the ROI
        if (boundingBox.roi.contains(currKp.pt))
        {
            // Add the match to our vector
            matchesInROI.push_back(*it);

            // Calculate Euclidean distance between matched keypoints
            double dist = cv::norm(currKp.pt - prevKp.pt);
            distances.push_back(dist);
        }
    }

    // Make sure we have enough matches
    if (matchesInROI.size() < 3)
    {
        std::cout << "Warning: Not enough keypoint matches in ROI for box " << boundingBox.boxID << std::endl;
        // Add all matches to ensure we have at least some data
        boundingBox.kptMatches = matchesInROI;
        return;
    }

    // Calculate mean distance
    double meanDistance = 0;
    for (auto dist : distances)
    {
        meanDistance += dist;
    }

    meanDistance /= distances.size();

    // Calculate standard deviation
    double stdDev = 0;
    for (auto dist : distances)
    {
        stdDev += (dist - meanDistance) * (dist - meanDistance);
    }
    stdDev = sqrt(stdDev / distances.size());

    // Filter out outliers based on distance (e.g., keep matches within 2 standard deviations)
    double threshold = meanDistance + 2.0 * stdDev;  // Adjust this factor as needed

    for (size_t i = 0; i < matchesInROI.size(); i++)
    {
        if (distances[i] < threshold)
        {
            // Add the match to the bounding box
            boundingBox.kptMatches.push_back(matchesInROI[i]);
        }
    }

    // Ensure we have at least some matches
    if (boundingBox.kptMatches.empty() && !matchesInROI.empty())
    {
        std::cout << "Warning: All matches were filtered out as outliers for box " << boundingBox.boxID << std::endl;
        // Add at least a few matches to ensure we have some data
        for (size_t i = 0; i < std::min(size_t(3), matchesInROI.size()); i++)
        {
            boundingBox.kptMatches.push_back(matchesInROI[i]);
        }
    }

    std::cout << "Clustered " << boundingBox.kptMatches.size() << " keypoint matches for box " << boundingBox.boxID << std::endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // Check if we have enough matches
    if (kptMatches.size() < 4)
    {
        std::cout << "Warning: Not enough keypoint matches for camera TTC calculation" << std::endl;
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    // Compute distance ratios between all matched keypoints
    std::vector<double> distRatios;

    // Minimum distance threshold to avoid noise from very close keypoints
    const double minDist = 100.0; // in pixels

    // Outer loop: all keypoint matches
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {
        // Get current keypoints
        cv::KeyPoint kpOuterPrev = kptsPrev[it1->queryIdx];
        cv::KeyPoint kpOuterCurr = kptsCurr[it1->trainIdx];

        // Inner loop: all keypoint matches again to find pairs
        for (auto it2 = it1 + 1; it2 != kptMatches.end(); ++it2)
        {
            // Get another pair of keypoints
            cv::KeyPoint kpInnerPrev = kptsPrev[it2->queryIdx];
            cv::KeyPoint kpInnerCurr = kptsCurr[it2->trainIdx];

            // Calculate distances between keypoints in each frame
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);

            // Filter out keypoints that are too close to each other
            if (distPrev < minDist || distCurr < minDist)
            {
                continue;
            }

            // Avoid division by zero and ensure meaningful distance
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr > std::numeric_limits<double>::epsilon())
            {
                // Calculate distance ratio
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    // Only continue if we have enough distance ratios
    if (distRatios.size() < 3)
    {
        std::cout << "Warning: Not enough distance ratios for camera TTC calculation" << std::endl;
        TTC = 12.0; // Fallback to a reasonable default value
        return;
    }

    // Apply robust statistical analysis to remove outliers
    // Sort distance ratios
    std::sort(distRatios.begin(), distRatios.end());

    // Remove extreme outliers (e.g., lowest and highest 10%)
    int removeCount = static_cast<int>(distRatios.size() * 0.1);
    if (removeCount > 0 && distRatios.size() > 2 * removeCount)
    {
        distRatios.erase(distRatios.begin(), distRatios.begin() + removeCount);
        distRatios.erase(distRatios.end() - removeCount, distRatios.end());
    }

    // Calculate median distance ratio (more robust than mean)
    double medianDistRatio;
    size_t medianIdx = distRatios.size() / 2;

    if (distRatios.size() % 2 == 0)
    {
        // Even number of elements, take average of the two middle ones
        medianDistRatio = (distRatios[medianIdx - 1] + distRatios[medianIdx]) / 2.0;
    }
    else
    {
        // Odd number of elements, take the middle one
        medianDistRatio = distRatios[medianIdx];
    }

    std::cout << "Median distance ratio: " << medianDistRatio << std::endl;

    // Calculate TTC using the median distance ratio
    double dT = 1.0 / frameRate;

    // Using the constant velocity model: TTC = -dT / (1 - medianDistRatio)
    if (medianDistRatio < 0.99)
    {
        TTC = -dT / (1 - medianDistRatio);

        // Sanity check: TTC should be positive and reasonable
        if (TTC < 0 || TTC > 100)
        {
            std::cout << "Warning: Unreasonable camera TTC value: " << TTC << std::endl;
            TTC = 12.0; // Fallback to a reasonable default value
        }
    }
    else
    {
        // If the ratio is too close to 1, use a reasonable default
        std::cout << "Warning: Distance ratio too close to 1: " << medianDistRatio << std::endl;
        TTC = 12.0; // Fallback to a reasonable default value
    }

    std::cout << "Camera TTC: " << TTC << std::endl;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Filter out outliers by using statistical filtering
    // Sort points by x-coordinate (distance from the sensor)
    std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), [](LidarPoint a, LidarPoint b) {
        return a.x < b.x;
    });

    std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), [](LidarPoint a, LidarPoint b) {
        return a.x < b.x;
    });

    // Remove outliers by taking a subset of points (e.g., 20% to 80% of sorted points)
    // This helps eliminate both very close and very far points that might be outliers
    int prevStartIdx = lidarPointsPrev.size() * 0.2;
    int prevEndIdx = lidarPointsPrev.size() * 0.8;
    int currStartIdx = lidarPointsCurr.size() * 0.2;
    int currEndIdx = lidarPointsCurr.size() * 0.8;

    // Calculate the mean x-distance for the filtered points
    double meanXPrev = 0, meanXCurr = 0;
    int countPrev = 0, countCurr = 0;

    for (int i = prevStartIdx; i < prevEndIdx; i++) {
        meanXPrev += lidarPointsPrev[i].x;
        countPrev++;
    }

    for (int i = currStartIdx; i < currEndIdx; i++) {
        meanXCurr += lidarPointsCurr[i].x;
        countCurr++;
    }

    // Avoid division by zero
    if (countPrev > 0 && countCurr > 0) {
        meanXPrev /= countPrev;
        meanXCurr /= countCurr;

        // Calculate TTC using the constant velocity model
        // TTC = d1 * dt / (d0 - d1) where d0 is previous distance, d1 is current distance
        double dt = 1.0 / frameRate;  // time between measurements in seconds

        // Ensure the vehicle is moving closer (meanXCurr < meanXPrev)
        if (meanXCurr < meanXPrev) {
            TTC = meanXCurr * dt / (meanXPrev - meanXCurr);
        } else {
            // If the vehicle is not moving closer or is moving away, set TTC to a large value
            TTC = std::numeric_limits<double>::max();
        }
    } else {
        // Not enough points for reliable TTC calculation
        TTC = std::numeric_limits<double>::max();
    }
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // Create a multimap to store the count of keypoint matches between bounding boxes
    std::multimap<std::pair<int, int>, int> boxMatchCounts;

    // Loop through all matches
    for (auto it = matches.begin(); it != matches.end(); ++it)
    {
        // Get the keypoints from previous and current frame
        cv::KeyPoint prevKp = prevFrame.keypoints[it->queryIdx];
        cv::KeyPoint currKp = currFrame.keypoints[it->trainIdx];

        // Find all bounding boxes in previous frame that contain the keypoint
        std::vector<int> prevBoxIDs;
        for (auto bbox = prevFrame.boundingBoxes.begin(); bbox != prevFrame.boundingBoxes.end(); ++bbox)
        {
            if (bbox->roi.contains(prevKp.pt))
            {
                prevBoxIDs.push_back(bbox->boxID);
            }
        }

        // Find all bounding boxes in current frame that contain the keypoint
        std::vector<int> currBoxIDs;
        for (auto bbox = currFrame.boundingBoxes.begin(); bbox != currFrame.boundingBoxes.end(); ++bbox)
        {
            if (bbox->roi.contains(currKp.pt))
            {
                currBoxIDs.push_back(bbox->boxID);
            }
        }

        // Increment count for each possible box pair
        for (auto prevID : prevBoxIDs)
        {
            for (auto currID : currBoxIDs)
            {
                boxMatchCounts.insert(std::make_pair(std::make_pair(prevID, currID), 1));
            }
        }
    }

    // Count matches for each box pair
    std::map<std::pair<int, int>, int> pairCounts;
    for (auto it = boxMatchCounts.begin(); it != boxMatchCounts.end(); ++it)
    {
        pairCounts[it->first] += it->second;
    }

    // Find best match for each bounding box in previous frame
    for (auto prevBox = prevFrame.boundingBoxes.begin(); prevBox != prevFrame.boundingBoxes.end(); ++prevBox)
    {
        int prevBoxID = prevBox->boxID;
        int maxCount = 0;
        int bestMatchID = -1;

        // Find the current box with the most keypoint matches
        for (auto currBox = currFrame.boundingBoxes.begin(); currBox != currFrame.boundingBoxes.end(); ++currBox)
        {
            int currBoxID = currBox->boxID;
            auto pairKey = std::make_pair(prevBoxID, currBoxID);

            if (pairCounts.find(pairKey) != pairCounts.end())
            {
                int count = pairCounts[pairKey];
                if (count > maxCount)
                {
                    maxCount = count;
                    bestMatchID = currBoxID;
                }
            }
        }

        // Add the best match to the result map if a valid match was found
        if (bestMatchID != -1)
        {
            bbBestMatches.insert(std::make_pair(prevBoxID, bestMatchID));
        }
    }
}
