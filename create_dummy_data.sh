#!/bin/bash

# Create directories
mkdir -p images/KITTI/2011_09_26/image_02/data
mkdir -p images/KITTI/2011_09_26/velodyne_points/data

# Create dummy image files (1x1 pixel black images)
for i in {0..19}; do
    # Format number with leading zeros (10 digits)
    num=$(printf "%010d" $i)
    
    # Create a simple 1x1 pixel black PNG image
    convert -size 1242x375 xc:black images/KITTI/2011_09_26/image_02/data/${num}.png
    
    echo "Created images/KITTI/2011_09_26/image_02/data/${num}.png"
done

# Create dummy LiDAR files (empty binary files)
for i in {0..19}; do
    # Format number with leading zeros (10 digits)
    num=$(printf "%010d" $i)
    
    # Create an empty binary file
    dd if=/dev/zero of=images/KITTI/2011_09_26/velodyne_points/data/${num}.bin bs=1 count=1000
    
    echo "Created images/KITTI/2011_09_26/velodyne_points/data/${num}.bin"
done

echo "Dummy data creation complete!"
