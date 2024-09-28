# Simultaneous Localization and Mapping Algorithm


This document outlines a simultaneous localization and mapping (SLAM) algorithm designed for use with 2D scans collected from a Unity simulation environment. In this setup, a LiDAR scanner is attached to a motor (such as a stepper motor), which facilitates the collection of 360-degree LiDAR data represented as angles (ranging from 0 to 360 degrees) and distances.

Overview of the Algorithm
The core of the algorithm is to compare and find distances between points in the collected scans, using a nearest neighbors algorithm coupled with the Iterative Closest Point (ICP) algorithm for matching. The overall workflow consists of the following key steps:

Data Acquisition: Scans are obtained from the simulation environment, where the LiDAR continuously captures 360-degree data.

Finding Nearest Neighbors: Using a KDTree, the algorithm identifies the nearest neighbors of points in the current scan to those in the previous scan. This is crucial for aligning the point clouds accurately.

Filtering Points: Close points from the aligned scans are filtered based on a specified distance threshold to ensure that only relevant points contribute to the mapping process.

ICP Transformation Calculation: The algorithm computes the transformation (rotation and translation) needed to align the current scan with the previous one. This is done by calculating centroids, centering the point clouds, and using Singular Value Decomposition (SVD) to derive the optimal rotation and translation.

RANSAC Integration: The Random Sample Consensus (RANSAC) method is employed to robustly estimate the transformation, filtering out outliers that could corrupt the alignment.

Iterative Refinement: The algorithm iteratively refines the alignment of the point clouds using the results from the ICP algorithm and RANSAC, continually updating the transformation until convergence is achieved.

Loop Closure Detection: As the robot moves, the algorithm detects potential loop closures by measuring distances between the current pose and previously recorded poses, allowing for corrections in trajectory and mapping.

Pose Graph Optimization: To enhance global map consistency, a pose graph optimization process is performed, utilizing constraints derived from loop closures. The optimizer updates the trajectory based on these constraints, refining the poses of the robot in the global context.

Global Map Update: The global map is updated with points from the scans transformed according to the optimized poses, ensuring that new data is accurately integrated into the existing map.

Visualization: The algorithm incorporates real-time visualization using PyQtGraph, displaying the global map and trajectory of the robot as it navigates the environment.
