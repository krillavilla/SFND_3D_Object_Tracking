#!/usr/bin/env python3
"""
Script to download and extract a subset of the KITTI dataset for the 3D Object Tracking project.
This script downloads only the necessary files for the project to save bandwidth and disk space.
"""

import os
import sys
import requests
import zipfile
import io
import argparse
from tqdm import tqdm

def download_file(url, destination):
    """
    Download a file from a URL with a progress bar.
    
    Args:
        url: URL to download from
        destination: Path to save the file to
    """
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(destination), exist_ok=True)
    
    # Download with progress bar
    response = requests.get(url, stream=True)
    total_size = int(response.headers.get('content-length', 0))
    block_size = 1024  # 1 Kibibyte
    
    with open(destination, 'wb') as f, tqdm(
            desc=os.path.basename(destination),
            total=total_size,
            unit='iB',
            unit_scale=True,
            unit_divisor=1024,
        ) as bar:
        for data in response.iter_content(block_size):
            size = f.write(data)
            bar.update(size)

def main():
    parser = argparse.ArgumentParser(description='Download KITTI dataset for 3D Object Tracking')
    parser.add_argument('--output-dir', default='images/KITTI', help='Output directory for KITTI dataset')
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # URLs for the KITTI dataset files we need
    # Note: These are not the official KITTI URLs, but a subset hosted for this project
    # In a real scenario, you would need to register on the KITTI website and download from there
    urls = {
        'camera': 'https://s3.amazonaws.com/udacity-sdc/SFND/KITTI_sample/2011_09_26_drive_0005_sync_image_02.zip',
        'lidar': 'https://s3.amazonaws.com/udacity-sdc/SFND/KITTI_sample/2011_09_26_drive_0005_sync_velodyne_points.zip'
    }
    
    # Download and extract camera images
    print("Downloading camera images...")
    camera_zip = os.path.join(args.output_dir, 'camera.zip')
    download_file(urls['camera'], camera_zip)
    
    print("Extracting camera images...")
    with zipfile.ZipFile(camera_zip, 'r') as zip_ref:
        zip_ref.extractall(args.output_dir)
    
    # Download and extract LiDAR data
    print("Downloading LiDAR data...")
    lidar_zip = os.path.join(args.output_dir, 'lidar.zip')
    download_file(urls['lidar'], lidar_zip)
    
    print("Extracting LiDAR data...")
    with zipfile.ZipFile(lidar_zip, 'r') as zip_ref:
        zip_ref.extractall(args.output_dir)
    
    # Clean up zip files
    os.remove(camera_zip)
    os.remove(lidar_zip)
    
    # Verify the extraction
    camera_dir = os.path.join(args.output_dir, '2011_09_26/image_02/data')
    lidar_dir = os.path.join(args.output_dir, '2011_09_26/velodyne_points/data')
    
    if os.path.exists(camera_dir) and os.path.exists(lidar_dir):
        camera_files = len(os.listdir(camera_dir))
        lidar_files = len(os.listdir(lidar_dir))
        print(f"Successfully extracted {camera_files} camera images and {lidar_files} LiDAR files.")
        print("KITTI dataset is ready for use with the 3D Object Tracking project.")
    else:
        print("Error: Extraction failed. Please check the downloaded files.")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
