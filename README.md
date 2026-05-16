## Overview
This project implements a complete pipeline for processing and analyzing 3D LiDAR point cloud data from vehicle-mounted sensors. The system performs two critical tasks for autonomous vehicle perception:

Ground Plane Segmentation - Separates the road surface from other objects
Obstacle Detection - Identifies and locates nearby objects and hazards

These techniques are fundamental to autonomous driving systems, robotics, and environmental mapping applications.
## Demo
### ✨ Core Capabilities:
1. Real-time ground plane segmentation using geometric algorithms
2. Fast obstacle detection and clustering
3. Point cloud visualization and analysis
4. Noise filtering and data preprocessing
5. Support for multiple LiDAR data formats
## Prerequisites
Before you begin, ensure you have:

1. MATLAB R2020a or later
2. Lidar Toolbox (optional, for enhanced visualization)
3. Computer Vision Toolbox (recommended)
4. A LiDAR dataset in MAT, PCD, or text format
## Getting Started
### Quick Start (30 seconds)
```text
% Load sample LiDAR data
load('your_lidar_data.mat');

% Run the segmentation pipeline
[ground, obstacles] = segmentPointCloud(pointCloud);

% Visualize results
visualizeSegmentation(ground, obstacles);
source install/setup.bash
```
## Advanced Usage
### Custom Parameter Configuration
```text
% Define custom parameters
params.groundThreshold = 0.15;      % Ground fitting tolerance (m)
params.clusterMinSize = 10;         % Minimum obstacle cluster size
params.maxHeight = 2.5;             % Maximum object height (m)

% Run with custom parameters
[ground, obstacles] = lidarSegmentation(ptCloud, params);
```
## Performance Metrics
```text
% Calculate segmentation quality
metrics = evaluateSegmentation(groundTruth, predictedSegmentation);

fprintf('Precision: %.3f\n', metrics.precision);
fprintf('Recall: %.3f\n', metrics.recall);
fprintf('F1-Score: %.3f\n', metrics.f1Score);
```
## Image
<img width="700" height="525" alt="Point Cloud" src="https://github.com/user-attachments/assets/9e61e26b-8a3f-4d51-845b-b2746bba3cef" />
