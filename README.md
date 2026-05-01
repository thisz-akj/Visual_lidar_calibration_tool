# Visual LiDAR Calibration Tool

## Overview
This repository implements a targetless camera–LiDAR extrinsic calibration pipeline designed for real-world robotic and autonomous systems. The system estimates the rigid transformation between a monocular camera and a 3D LiDAR sensor using synchronized ROS2 bag data, eliminating the need for calibration targets.

The pipeline integrates learned feature matching, geometric optimization, and statistical refinement to produce robust and accurate calibration under challenging conditions such as noise, sparsity, dynamic scenes, and varying illumination.

---

## Problem Statement
Accurate extrinsic calibration between camera and LiDAR is critical for multi-sensor perception systems used in robotics and ADAS. Traditional calibration methods rely on calibration targets, which are impractical in production environments.

This project addresses:
- Targetless calibration in unstructured environments  
- Robust alignment under noisy and sparse LiDAR data  
- Real-world deployment using ROS2 sensor pipelines  

---

## Key Features

### Targetless Calibration
- No checkerboards or fiducial markers required  
- Works directly with natural scene geometry  

### ROS2-Based Sensor Pipeline
- Uses ROS2 bag files as primary input  
- Supports standard topics:
  - `/camera/image_raw`
  - `/lidar/points`  
- Designed for real robotic deployments  

### Learned Feature Matching
- SuperGlue-based matching for robust correspondence  
- Handles viewpoint variation and low-texture regions  

### Multi-Stage Optimization Pipeline
- RANSAC for outlier rejection  
- SVD-based rigid transformation initialization  
- Non-linear optimization using Ceres Solver  
- Final refinement using Normalized Information Distance (NID)  

### Cross-Modal Validation
- Projects LiDAR point clouds into image space  
- Evaluates calibration using:
  - segmentation alignment  
  - centroid consistency  

## Results

<img width="581" height="367" alt="image" src="https://github.com/user-attachments/assets/1cfce0d2-1bd8-4906-b6da-4f01d737f1c5" />

<img width="522" height="344" alt="image" src="https://github.com/user-attachments/assets/fa01da7c-16a8-4230-9721-47ea01779778" />

<img width="422" height="449" alt="image" src="https://github.com/user-attachments/assets/04d8a4e1-3fb3-4cde-a564-4612fea27c93" />
<img width="422" height="449" alt="image" src="https://github.com/user-attachments/assets/fa7cd41a-ec2d-4144-b555-cdaf4a3ddca4" />




---

