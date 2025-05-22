#!/usr/bin/env python3
import os
import numpy as np
import cv2

# Define paths
base_dir = os.path.dirname(os.path.abspath(__file__))
image_dir = os.path.join(base_dir, "images/KITTI/2011_09_26/image_02/data")
lidar_dir = os.path.join(base_dir, "images/KITTI/2011_09_26/velodyne_points/data")

# Create directories if they don't exist
os.makedirs(image_dir, exist_ok=True)
os.makedirs(lidar_dir, exist_ok=True)

# Create dummy image files (black images with frame number)
for i in range(20):  # Create 20 frames (0-19)
    # Create a black image
    img = np.zeros((375, 1242, 3), dtype=np.uint8)
    
    # Add frame number text
    cv2.putText(img, f"Frame {i}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Add some random rectangles to simulate objects
    for _ in range(5):
        x1 = np.random.randint(100, 1000)
        y1 = np.random.randint(100, 300)
        w = np.random.randint(50, 200)
        h = np.random.randint(50, 100)
        color = tuple(np.random.randint(0, 255, 3).tolist())
        cv2.rectangle(img, (x1, y1), (x1 + w, y1 + h), color, 2)
    
    # Save the image
    filename = f"{image_dir}/{i:010d}.png"
    cv2.imwrite(filename, img)
    print(f"Created {filename}")

# Create dummy LiDAR files (random point clouds)
for i in range(20):  # Create 20 frames (0-19)
    # Create random point cloud data (x, y, z, intensity)
    num_points = 10000  # 10k points per frame
    points = np.random.randn(num_points, 4).astype(np.float32)
    
    # Make it somewhat realistic
    # x: forward (5-20m)
    points[:, 0] = np.random.uniform(5, 20, num_points)
    # y: left/right (-5 to 5m)
    points[:, 1] = np.random.uniform(-5, 5, num_points)
    # z: height (-1 to 1m)
    points[:, 2] = np.random.uniform(-1, 1, num_points)
    # intensity: (0-1)
    points[:, 3] = np.random.uniform(0, 1, num_points)
    
    # Add some "car-like" clusters
    for _ in range(3):
        center_x = np.random.uniform(8, 15)
        center_y = np.random.uniform(-3, 3)
        center_z = -0.5
        
        # Create cluster of points
        cluster_size = 500
        cluster_points = np.random.randn(cluster_size, 4).astype(np.float32) * 0.5
        cluster_points[:, 0] += center_x
        cluster_points[:, 1] += center_y
        cluster_points[:, 2] += center_z
        cluster_points[:, 3] = np.random.uniform(0.5, 1, cluster_size)
        
        # Replace some points with cluster
        start_idx = np.random.randint(0, num_points - cluster_size)
        points[start_idx:start_idx+cluster_size] = cluster_points
    
    # Save the point cloud
    filename = f"{lidar_dir}/{i:010d}.bin"
    points.tofile(filename)
    print(f"Created {filename}")

print("Dummy data creation complete!")
