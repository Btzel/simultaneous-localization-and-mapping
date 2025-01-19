# 2D LiDAR SLAM Implementation

A Python implementation of Simultaneous Localization and Mapping (SLAM) algorithm using 2D LiDAR scan data from Unity simulation environment.

![Python](https://img.shields.io/badge/Python-3.8+-blue)
![NumPy](https://img.shields.io/badge/NumPy-1.19+-green)
![PyQt5](https://img.shields.io/badge/PyQt5-5.15+-orange)
![g2o](https://img.shields.io/badge/g2o-latest-red)

## 🎯 Overview

This SLAM implementation processes 360-degree LiDAR scans to create a map while simultaneously tracking the robot's position. The system uses advanced point cloud registration techniques and pose graph optimization for accurate mapping.

## 🔧 Technical Components

### Data Processing Pipeline

1. **Data Acquisition**
```python
def acquisition():
    with open(r'LidarData.txt', 'r') as file:
        lidar_dataset = file.read()
    dataset = lidar_dataset.split(';|\n')
    return np.array(dataset)
```

2. **Point Cloud Registration**
- KDTree-based nearest neighbor search
- Iterative Closest Point (ICP) algorithm
- RANSAC for robust transformation estimation

3. **Loop Closure**
- Trajectory analysis for loop detection
- Pose graph optimization using g2o
- Global map consistency maintenance

## 💻 Implementation Details

### Core Algorithms

#### ICP with RANSAC
```python
def iterative_icp_with_ransac(P, Q, max_iterations=50, tolerance=1e-18):
    """
    P: Reference point cloud
    Q: Current point cloud to be aligned
    """
```

#### Pose Graph Optimization
```python
def optimize_pose_graph(trajectory, constraints):
    """
    Optimize robot trajectory using g2o
    """
```

### Key Features

1. **Point Cloud Processing**
   - Efficient nearest neighbor search
   - Point filtering for noise reduction
   - Transformation computation

2. **Mapping**
   - Real-time map updates
   - Global map consistency
   - Loop closure handling

3. **Visualization**
   - PyQtGraph-based real-time display
   - Trajectory visualization
   - Map point cloud rendering

## 🛠️ Dependencies

- numpy: Matrix operations and numerical computations
- scipy: KDTree implementation for nearest neighbor search
- PyQt5: GUI framework
- pyqtgraph: Real-time visualization
- g2o: Pose graph optimization

## 🚀 Usage

1. Install dependencies:
```bash
pip install numpy scipy PyQt5 pyqtgraph g2o
```

2. Prepare LiDAR data:
- Place your LiDAR scan data in `LidarData.txt`
- Format: angle:distance pairs separated by semicolons

3. Run the SLAM algorithm:
```bash
python slam.py
```

## 📊 Visualization

The system provides real-time visualization showing:
- Current LiDAR scan points
- Accumulated global map
- Robot trajectory
- Optimized path after loop closure

## 🔍 Algorithm Details

### 1. Data Processing
- Conversion from polar to Cartesian coordinates
- Noise filtering
- Point cloud preprocessing

### 2. Scan Matching
```python
def find_nearest_neighbors_kdtree(P, Q):
    """Find nearest points in P for each point in Q using KDTree."""
    tree = KDTree(P)
    nearest_neighbors = np.zeros(Q.shape[0], dtype=int)
    for i, q in enumerate(Q):
        _, nearest_neighbors[i] = tree.query(q)
    return nearest_neighbors
```

### 3. Loop Closure Detection
```python
def detect_loop_closure(current_pose, trajectory):
    """Check for loop closure in the trajectory."""
    for index, pose in enumerate(trajectory):
        distance = np.linalg.norm(current_pose - pose[:2])
        if distance < 10:  # Threshold for loop closure
            return index
    return None
```

## 🔄 Future Improvements

1. **Sensor Fusion**
   - Integration with IMU data
   - Multiple sensor support
   - Enhanced pose estimation

2. **Performance Optimization**
   - Parallel processing implementation
   - Memory usage optimization
   - Real-time performance enhancement

3. **Feature Enhancement**
   - 3D SLAM support
   - Dynamic environment handling
   - Advanced loop closure detection

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
1. Sensor fusion implementation
2. Performance optimization
3. Additional feature development
4. Documentation enhancement

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Author
Burak TÜZEL

## 📚 References

1. ICP Algorithm: Besl, P.J. and McKay, N.D., 1992
2. RANSAC: Fischler and Bolles, 1981
3. G2O: General Graph Optimization
4. KDTree Implementation: scipy.spatial
