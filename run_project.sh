#!/bin/bash

# Exit on error
set -e

echo "Setting up and running the 3D Object Tracking project..."

# Download YOLO weights if they don't exist
echo "Checking for YOLO weights files..."
if [ ! -f "dat/yolo/yolov3.weights" ]; then
    echo "Downloading yolov3.weights..."
    wget -O dat/yolo/yolov3.weights https://pjreddie.com/media/files/yolov3.weights
fi

# Check if KITTI dataset exists
echo "Checking for KITTI dataset..."
if [ ! -d "images/KITTI/2011_09_26/image_02/data" ] || [ ! -d "images/KITTI/2011_09_26/velodyne_points/data" ]; then
    echo "KITTI dataset not found. Creating dummy data for testing..."
    
    # Create directories if they don't exist
    mkdir -p images/KITTI/2011_09_26/image_02/data
    mkdir -p images/KITTI/2011_09_26/velodyne_points/data
    
    # Run the dummy data creation script if it exists
    if [ -f "create_dummy_data.py" ]; then
        echo "Running create_dummy_data.py to generate dummy data..."
        python3 create_dummy_data.py
    else
        echo "WARNING: create_dummy_data.py not found. You may need to download the KITTI dataset manually."
    fi
fi

# Remove existing build directory to avoid CMake cache issues
echo "Removing existing build directory..."
rm -rf build

# Create a fresh build directory
mkdir -p build && cd build

# Run CMake and make
echo "Running CMake..."
cmake ..

echo "Building the application..."
make -j$(nproc)

# Check if build was successful
if [ ! -f "./3D_object_tracking" ]; then
    echo "Error: Build failed. The executable was not created."
    exit 1
fi

echo "Build successful!"

# Copy YOLO config files if they don't exist
if [ ! -f "coco.names" ]; then
    echo "Copying YOLO config files..."
    cp ../dat/yolo/coco.names .
    cp ../dat/yolo/yolov3.cfg .
fi

# Copy YOLO weights if they don't exist in the build directory
if [ ! -f "yolov3.weights" ]; then
    echo "Copying YOLO weights to build directory..."
    cp ../dat/yolo/yolov3.weights .
fi

# Run the application with camera sweep to test all detector/descriptor combinations
echo "Running the application with camera sweep..."
./3D_object_tracking --camera_sweep --log results_full.csv

# Check if the results file was created
if [ ! -f "results_full.csv" ]; then
    echo "Error: Application did not generate results_full.csv"
    exit 1
fi

echo "Application run complete!"
echo "Results saved to build/results_full.csv"

# Generate a summary of the results
echo "Generating results summary..."
echo "Top 5 detector/descriptor combinations by TTC accuracy:"
head -n 1 results_full.csv > header.csv
sort -t, -k4,4n results_full.csv | grep -v "detector,descriptor" | head -n 5 > top5.csv
cat header.csv top5.csv

echo "Complete! You can now analyze the results to verify the project requirements."
echo "To analyze the results in detail, run: python3 ../analyze_results.py results_full.csv"
