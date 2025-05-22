#!/bin/bash

# Cleanup script for 3D Object Tracking project submission

echo "Starting repository cleanup..."

# Create backup of important files
echo "Creating backup of YOLO configuration files..."
mkdir -p backup/dat/yolo
cp dat/yolo/coco.names backup/dat/yolo/
cp dat/yolo/yolov3.cfg backup/dat/yolo/
cp dat/yolo/yolov3-tiny.cfg backup/dat/yolo/

# Backup important images for README
echo "Creating backup of important images..."
mkdir -p backup/images
cp images/FP5_*.png backup/images/ 2>/dev/null
cp images/FP6_*.png backup/images/ 2>/dev/null
cp images/FP6_comparison_table.csv backup/images/ 2>/dev/null
cp images/course_code_structure.png backup/images/ 2>/dev/null

# Remove build artifacts
echo "Removing build artifacts..."
rm -rf build/
rm -rf cmake-build-debug/

# Remove large binary files but keep a note about them
echo "Removing large binary files..."
rm -f dat/yolo/*.weights
echo "# YOLO weights files were removed to reduce repository size" > dat/yolo/README_WEIGHTS.md
echo "Download the following files and place them in this directory:" >> dat/yolo/README_WEIGHTS.md
echo "1. yolov3.weights: https://pjreddie.com/media/files/yolov3.weights" >> dat/yolo/README_WEIGHTS.md
echo "2. yolov3-tiny.weights: https://pjreddie.com/media/files/yolov3-tiny.weights" >> dat/yolo/README_WEIGHTS.md

# Remove KITTI dataset files but keep a note about them
echo "Removing KITTI dataset files..."
rm -rf images/KITTI/
mkdir -p images/KITTI
echo "# KITTI dataset files were removed to reduce repository size" > images/KITTI/README_DATASET.md
echo "The program expects the following directory structure:" >> images/KITTI/README_DATASET.md
echo "- images/KITTI/2011_09_26/image_02/data/*.png (camera images)" >> images/KITTI/README_DATASET.md
echo "- images/KITTI/2011_09_26/velodyne_points/data/*.bin (LiDAR data)" >> images/KITTI/README_DATASET.md
echo "Download the KITTI dataset from: https://www.cvlibs.net/datasets/kitti/raw_data.php" >> images/KITTI/README_DATASET.md

# Remove duplicate or unnecessary source files
echo "Removing duplicate or unnecessary source files..."
rm -f camFusion_Student_implementations.cpp
rm -f evaluate_detector_descriptor.cpp
rm -f generate_lidar_images.cpp

# Remove temporary and script files, but keep useful ones
echo "Removing temporary and script files..."
rm -f build_and_run.sh
rm -f create_udacity_package.sh
rm -f generate_lidar_anomalies.sh
rm -f generate_lidar_simple.sh

# Keep plot_results.py if it's referenced in the README
if grep -q "plot_results.py" README.md; then
  echo "Keeping plot_results.py as it's referenced in README.md"
else
  rm -f plot_results.py
fi

# Remove duplicate README
echo "Removing duplicate README..."
rm -f README_new.md

# Remove any temporary files
echo "Removing temporary files..."
rm -f *.tmp
rm -f *.log
rm -f *~
rm -f *.bak

# Restore important files from backup
echo "Restoring important files from backup..."
cp -r backup/dat .
cp -r backup/images .
rm -rf backup

echo "Cleanup complete!"
echo "The repository is now ready for submission."
echo "Remember to check the final structure with: find . -type f | sort"
