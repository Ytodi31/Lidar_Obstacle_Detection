## LiDAR Obstacle Detection


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview
The project steams point cloud data captured by a LiDAR mounted on self-driving test vehicle in a narrow street in city environment. \
This project was completed as a part of Udacity's nano degree program on Sensor Fusion and used starter files.
Function implementations were done using in built functions from the Point Cloud Library (PCL), followed by self defined functions. An analysis is done between the two methodologies.

<p align="center">
  <img src="/data/lidar_xy_view.gif" width="512" height = "256" title="Detection of Obstacle from LiDAR data">
</p>

## Dependencies
The project was developed in Linux environment with Ubuntu distribution.

The project extensively uses the Point Cloud Library.

To install : \
` sudo apt install libpcl-dev`

## Pipeline

* **Downsampling & Filtering** : To make the pipeline efficient, the point cloud data was down sampled using the Voxel filter and a region of interest was extracted which focused on the objects on the road.
* **Segmentation** : The down sampled data was segmented using RANSAC to create two point clouds - obstacle point cloud (on-coming cars) and plane point cloud (road)
* **Clustering** : After segmentation, the next step was to cluster the obstacles. In order to cluster the obstacles, the obstacle cloud was iterated and searched for nearest neighbors using KD Tree.
* **Bounding box & stream** : Next, a bounding box was created around each cluster and stream of point cloud data was processed through this pipeline
* **Comparison** : Segmentation and Clustering were first performed using the in-built functions from the point cloud library. Later, self-implemented functions were added to perform segmentation and clustering. This is shown in the results.



## Build and Run

`git clone https://github.com/Ytodi31/Lidar_Obstacle_Detection` \
` cd Lidar_Obstacle_Detection` \
` mkdir build ` \
` cd  build` \
`cmake ..` \
`make` \
`./enviroment`


## Results

This shows the result from using the in-built functions of the Point Cloud Library:
<p align="center">
  <img src="/data/pcl_fps.gif" width="512" height = "256" title="Detection of Obstacle from LiDAR data">
</p>

This shows the result from implementing the user defined functions for Clustering and Segmentation:

<p align="center">
  <img src="/data/lidar_fps_view.gif" width="512" height = "256" title="Detection of Obstacle from LiDAR data">
</p>
