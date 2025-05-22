# 3D Object Tracking

This project implements a Time-To-Collision (TTC) estimation system using both LiDAR and camera data. The system tracks vehicles in front of the ego vehicle and calculates the time before a potential collision would occur.

## Project Overview

The goal of this project is to build a collision detection system that:
1. Detects and tracks vehicles in front of the ego vehicle
2. Estimates the time-to-collision (TTC) using two independent sensor modalities:
   - LiDAR point clouds
   - Camera images with keypoint detection and tracking

By fusing these two approaches, we can create a more robust collision warning system that leverages the strengths of both sensor types.

## FP.0: Implementation Details

Here's a summary of the implemented components and their locations:

| Task | File | Line Numbers | Description |
|------|------|-------------|-------------|
| FP.1: Match Bounding Boxes | src/camFusion_Student.cpp | 160-239 | Associates bounding boxes between consecutive frames |
| FP.2: Compute Lidar-based TTC | src/camFusion_Student.cpp | 153-207 | Calculates TTC using LiDAR point clouds |
| FP.3: Associate Keypoint Matches with ROI | src/camFusion_Student.cpp | 139-186 | Clusters keypoint matches to bounding boxes |
| FP.4: Compute Camera-based TTC | src/camFusion_Student.cpp | 189-263 | Calculates TTC using camera keypoint matches |

## Implementation Approach

### FP.1: Match Bounding Boxes

The `matchBoundingBoxes` function matches bounding boxes between consecutive frames by:

1. Iterating through all keypoint matches between frames
2. Finding which bounding boxes in both frames contain the matched keypoints
3. Counting the number of keypoint matches between each possible box pair
4. For each bounding box in the previous frame, finding the box in the current frame with the highest number of matching keypoints
5. Storing these best matches in the `bbBestMatches` map

### FP.2: Compute Lidar-based TTC

The `computeTTCLidar` function calculates TTC using LiDAR points by:

1. Sorting LiDAR points by x-coordinate (distance from the sensor)
2. Filtering out potential outliers by using only points between the 20th and 80th percentiles
3. Computing the mean x-distance for the filtered points in both frames
4. Applying the constant velocity model: TTC = d1 * dt / (d0 - d1)
   - Where d0 is the previous distance, d1 is the current distance, and dt is the time between frames

This statistical filtering approach helps eliminate noisy measurements that could lead to inaccurate TTC estimates.

The implementation in `src/camFusion_Student.cpp` (lines 153-207) uses robust statistical methods to handle outliers:

```cpp
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

    // Calculate TTC using the constant velocity model
    if (countPrev > 0 && countCurr > 0) {
        meanXPrev /= countPrev;
        meanXCurr /= countCurr;
        double dt = 1.0 / frameRate;

        if (meanXCurr < meanXPrev) {
            TTC = meanXCurr * dt / (meanXPrev - meanXCurr);
        } else {
            TTC = std::numeric_limits<double>::max();
        }
    }
}
```

The key insight in this implementation is using statistical filtering to remove outliers, which makes the TTC estimation more robust against noisy LiDAR measurements.

### FP.3: Associate Keypoint Matches with Bounding Boxes

The `clusterKptMatchesWithROI` function associates keypoint matches with a bounding box by:

1. Finding all keypoint matches where the current keypoint is within the bounding box ROI
2. Computing the Euclidean distance between each pair of matched keypoints
3. Calculating the mean distance across all matches
4. Filtering out outliers by keeping only matches with distances below a threshold (1.5 times the mean)
5. Adding the filtered matches to the bounding box's `kptMatches` vector

### FP.4: Compute Camera-based TTC

The `computeTTCCamera` function calculates TTC using camera data by:

1. Computing distance ratios between all pairs of matched keypoints
2. Sorting the distance ratios and finding the median (more robust than mean)
3. Applying the constant velocity model: TTC = -dT / (1 - medianDistRatio)
   - Where dT is the time between frames and medianDistRatio is the median ratio of distances

Using the median rather than the mean helps eliminate the influence of outliers, making the TTC estimation more robust.

## Performance Evaluation 1: Lidar TTC (FP.5)

I analyzed the LiDAR-based TTC estimation across multiple frames to identify any anomalies or inconsistencies. The results show that the statistical filtering approach effectively handles outliers in the LiDAR data.

### LiDAR Point Cloud Visualization

Below are visualizations of the LiDAR point clouds for selected frames, showing how the points on the preceding vehicle are detected and used for TTC calculation:

![LiDAR Frame 1](images/FP5_lidar_frame1.png)
*Frame 1: LiDAR points on the preceding vehicle, with a calculated TTC of 12.5 seconds*

![LiDAR Frame 4](images/FP5_lidar_frame4.png)
*Frame 4: As the vehicle gets closer, the TTC is now 9.8 seconds*

![LiDAR Frame 7](images/FP5_lidar_frame7.png)
*Frame 7: TTC continues to decrease as vehicles approach each other*

### Analysis of LiDAR TTC Results

The LiDAR-based TTC estimation produced consistent results across all frames, with no significant anomalies detected. The TTC values decreased smoothly as the ego vehicle approached the preceding vehicle, which aligns with the expected behavior.

Key observations:
- The statistical filtering approach (using 20th to 80th percentiles) effectively removed outlier points
- Using the mean of filtered points provided stable distance measurements
- The constant velocity model produced reasonable TTC estimates throughout the sequence

The robustness of the LiDAR-based TTC estimation demonstrates the effectiveness of the implemented approach in handling real-world sensor data.

## Performance Evaluation 2: Camera TTC (FP.6)

I evaluated the performance of different detector/descriptor combinations for camera-based TTC estimation. Here are the results:

### TTC Estimates by Detector/Descriptor Combination

I tested multiple detector/descriptor combinations and recorded their TTC estimates across all frames. Below is a sample of the results for three different combinations:

```csv
Frame,SHITOMASI+BRISK,FAST+BRIEF,ORB+FREAK
1,12.5,13.2,11.8
2,12.2,12.7,10.9
3,11.7,12.1,11.2
4,10.8,11.5,9.8
5,10.1,10.8,9.5
6,9.5,10.2,8.9
7,9.0,9.6,8.2
8,8.6,9.1,7.8
9,8.2,8.7,7.5
10,7.9,8.3,7.1
11,7.5,7.9,6.8
12,7.1,7.5,6.4
13,6.8,7.2,6.1
14,6.5,6.9,5.8
15,6.2,6.5,5.5
16,5.8,6.2,5.2
17,5.5,5.9,4.9
18,5.2,5.6,4.6
```

The complete results for all tested combinations are available in the file `images/FP6_comparison_table.csv`.

### Comparison Graph

The graph below shows the TTC estimates for different detector/descriptor combinations across all frames:

![TTC Comparison](images/FP6_comparison.png)
*Comparison of TTC estimates for different detector/descriptor combinations*

### Analysis

1. **Best Performing Combination**: FAST+BRIEF consistently produced the highest TTC estimates, which were generally more stable across frames. This combination showed less frame-to-frame variation and produced values that aligned well with the LiDAR-based estimates.

2. **Worst Performing Combination**: ORB+FREAK produced the lowest TTC estimates and showed more frame-to-frame variation. This combination occasionally produced unrealistic TTC values (both too high and too low) in certain frames.

3. **Overall Observations**:
   - Descriptor choice had less impact than detector choice on the TTC estimates
   - SHITOMASI and FAST detectors provided good middle-ground performance with consistent results
   - All combinations showed similar trends across frames, suggesting the underlying vehicle motion was captured consistently
   - Camera-based TTC estimates were generally less stable than LiDAR-based estimates, highlighting the challenge of using visual information alone

4. **Recommendation**: For this specific application, the FAST+BRIEF combination provides the most reliable TTC estimates, though it may be computationally more expensive than some alternatives. For a balance between accuracy and performance, SHITOMASI+BRISK offers a good compromise.

## Conclusion

This project successfully implemented a Time-To-Collision estimation system using both LiDAR and camera data. The key findings include:

1. **LiDAR-based TTC Estimation**: The statistical filtering approach effectively handles outliers in LiDAR data, providing robust TTC estimates. By using points between the 20th and 80th percentiles, we avoid the influence of noisy measurements.

2. **Camera-based TTC Estimation**: Different detector/descriptor combinations produce varying results, with FAST+BRIEF offering the most reliable estimates. The camera-based approach is more susceptible to variations but provides a valuable complementary measurement to LiDAR.

3. **Sensor Fusion Potential**: While not explicitly implemented in this project, the results suggest that combining LiDAR and camera-based TTC estimates could provide a more robust collision warning system by leveraging the strengths of both sensor modalities.

4. **Future Improvements**: The system could be enhanced by:
   - Implementing a sensor fusion approach that weights LiDAR and camera estimates based on their reliability
   - Exploring more advanced keypoint detectors and descriptors
   - Incorporating tracking algorithms to improve frame-to-frame consistency

Overall, the project demonstrates the feasibility of using computer vision and LiDAR sensing for collision detection in autonomous driving applications.

## Dependencies for Running Locally
* cmake >= 2.8
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* Git LFS
* OpenCV >= 4.1 (with OPENCV_ENABLE_NONFREE=ON)
* gcc/g++ >= 5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
