from data import acquisition
import numpy as np
from scipy.spatial import KDTree
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
import g2o

data = acquisition()

def find_nearest_neighbors_kdtree(P, Q):
    """Find nearest points in P for each point in Q using KDTree."""
    tree = KDTree(P)
    nearest_neighbors = np.zeros(Q.shape[0], dtype=int)
    for i, q in enumerate(Q):
        _, nearest_neighbors[i] = tree.query(q)
    return nearest_neighbors

def filter_close_points(Q_aligned, global_map, distance_threshold):
    """Filter points in Q_aligned that are too close to global_map."""
    tree = KDTree(global_map)
    distances, _ = tree.query(Q_aligned)
    return Q_aligned[distances > distance_threshold]

def compute_icp_transformation(P, Q, corresponding_points):
    """Calculate ICP transformation (rotation and translation)."""
    centroid_P = np.mean(corresponding_points, axis=0)
    centroid_Q = np.mean(Q, axis=0)
    
    P_centered = corresponding_points - centroid_P
    Q_centered = Q - centroid_Q

    H = np.dot(Q_centered.T, P_centered)
    U, _, Vt = np.linalg.svd(H)
    
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = np.dot(Vt.T, U.T)
    
    t = centroid_P - np.dot(R, centroid_Q)
    return R, t

def ransac(P, Q, max_iterations=10, distance_threshold=0.05, min_inliers=0.5):
    """Find best transformation using RANSAC."""
    best_inliers = None
    best_transform = None
    max_inlier_count = 0

    for _ in range(max_iterations):
        indices = np.random.choice(len(Q), size=3, replace=False)
        sample_Q = Q[indices]
        sample_P = P[find_nearest_neighbors_kdtree(P, sample_Q)]

        R, t = compute_icp_transformation(sample_P, sample_Q, sample_P)

        Q_transformed = np.dot(Q, R.T) + t
        distances, _ = KDTree(P).query(Q_transformed)
        inliers = distances < distance_threshold
        inlier_count = np.sum(inliers)

        if inlier_count > max_inlier_count:
            max_inlier_count = inlier_count
            best_inliers = inliers
            best_transform = (R, t)

    if max_inlier_count / len(Q) < min_inliers:
        print("RANSAC failed: not enough inliers found.")
        return None, None

    return best_transform, best_inliers

def iterative_icp_with_ransac(P, Q, max_iterations=50, tolerance=1e-18, ransac_iterations=20):
    """Iteratively perform ICP with RANSAC for alignment."""
    prev_error = float('inf')
    R_total = np.eye(2)
    t_total = np.zeros((1, 2))
    inliers = None

    for iteration in range(max_iterations):
        (R, t), inliers = ransac(P, Q, max_iterations=ransac_iterations)

        if R is None or t is None:
            print("RANSAC failed, stopping ICP.")
            break

        Q_transformed = np.dot(Q, R.T) + t
        Q = Q_transformed[inliers]

        R_total = np.dot(R, R_total)
        t_total = t_total + np.dot(R_current, t_current.flatten())

        current_error = np.mean(np.linalg.norm(P[find_nearest_neighbors_kdtree(P, Q_transformed)] - Q_transformed, axis=1))
        print(f"Iteration {iteration + 1}: Error = {current_error}")

        if np.abs(prev_error - current_error) < tolerance:
            print(f"Converged after {iteration + 1} iterations with error: {current_error:.4f}.")
            break

        prev_error = current_error

    return Q_transformed, R_total, t_total, inliers

def detect_loop_closure(current_pose, trajectory):
    """Check for loop closure in the trajectory."""
    for index, pose in enumerate(trajectory):
        pose_position = pose[:2] if not isinstance(pose, g2o.g2opy.SE2) else np.array([pose.translation()[0], pose.translation()[1]])
        distance = np.linalg.norm(current_pose - pose_position)
        if distance < 10:  # Threshold for loop closure
            return index
    return None

def optimize_pose_graph(trajectory, constraints):
    """Optimize the pose graph using g2o."""
    optimizer = g2o.SparseOptimizer()
    linear_solver = g2o.BlockSolverX(g2o.LinearSolverDenseX())
    solver = g2o.OptimizationAlgorithmLevenberg(linear_solver)
    optimizer.set_algorithm(solver)
    optimizer.set_verbose(False)

    for vertex_id, pose in enumerate(trajectory):
        vertex = g2o.VertexSE2()
        vertex.set_id(vertex_id)

        if isinstance(pose, g2o.SE2):
            se2_pose = pose
        elif isinstance(pose, (list, tuple, np.ndarray)) and len(pose) in [2, 3]:
            se2_pose = g2o.SE2(*pose)  # x, y, theta
        else:
            raise ValueError("Invalid pose format.")

        vertex.set_estimate(se2_pose)
        optimizer.add_vertex(vertex)

    for (i, j, transform) in constraints:
        edge = g2o.EdgeSE2()
        edge.set_vertex(0, optimizer.vertex(i))
        edge.set_vertex(1, optimizer.vertex(j))
        
        measurement = g2o.SE2(*transform) if len(transform) in [2, 3] else None
        edge.set_measurement(measurement)
        optimizer.add_edge(edge)

    optimizer.initialize_optimization()
    optimizer.optimize(10000)

    for i in range(len(trajectory)):
        trajectory[i] = optimizer.vertex(i).estimate()

    return trajectory

def update_scan_with_pose_transformation(global_map, scan_pose_pairs, optimized_trajectory, raw_trajectory):
    """Update points in each scan with optimized poses."""
    transformed_points = np.empty((0, 2))
    for i, (raw_pose, points) in enumerate(scan_pose_pairs):
        raw_translation = raw_trajectory[i].translation()
        raw_rotation = raw_trajectory[i].rotation().angle()
        
        optimized_translation = optimized_trajectory[i].translation()
        optimized_rotation = optimized_trajectory[i].rotation().angle()

        delta_position = optimized_translation - raw_translation
        delta_rotation = optimized_rotation - raw_rotation

        rotation_matrix = np.array([[np.cos(delta_rotation), -np.sin(delta_rotation)],
                                     [np.sin(delta_rotation), np.cos(delta_rotation)]])

        transformed_points = np.dot(points, rotation_matrix.T) + delta_position

    global_map = np.vstack((global_map, transformed_points))
    
    return global_map


# Global optimization parameters
global_optimization_frequency = 1
global_map = np.empty((0, 2))
R_total = np.eye(2)
t_total = np.zeros((1, 2))

robot_position = np.array([0.0, 0.0])
robot_orientation = 0
raw_trajectory = []
optimized_trajectory = []
constraints = []
scan_pose_pairs = [] 

# Initialize PyQtGraph window
app = QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="LiDAR Mapping")
win.resize(800, 600)
plot = win.addPlot(title="LiDAR Mapping with Trajectory")
plot.setLabel('left', 'Y Position')
plot.setLabel('bottom', 'X Position')
global_map_item = plot.plot([], pen=pg.mkPen('b', width=1), symbol='o', symbolSize=2)  # Global map
raw_traj_item = plot.plot([], pen=pg.mkPen('orange', width=3), name='Raw Trajectory')  # Raw trajectory
opt_traj_item = plot.plot([], pen=pg.mkPen('g', width=2), name='Optimized Trajectory')  # Optimized trajectory

for i in range(len(data) - 1):
    if i < 100:
        continue
    
    P = data[i]
    Q = data[i + 1]
    
    # Perform ICP with RANSAC
    Q_aligned, R_current, t_current, best_inliers = iterative_icp_with_ransac(P, Q)
    
    if Q_aligned is not None:
        inliers = best_inliers
        # Transform the aligned points based on the current total rotation and translation
        Q_aligned = np.dot(Q_aligned, R_total.T) + t_total
        
        # Update total rotation and translation
        R_total = np.dot(R_current, R_total)
        t_total += np.dot(R_current, t_current.flatten())

        # Update robot position and orientation
        delta_position = t_current[0]
        robot_position += delta_position
        robot_orientation += np.arctan2(R_current[1, 0], R_current[0, 0])
        
        # Store the current position in the trajectory
        raw_trajectory.append(robot_position.copy())
        scan_pose_pairs.append((robot_position.copy(), Q_aligned[inliers]))
        
        # Update the global map with new points
        global_map = np.vstack((global_map, Q_aligned[inliers]))

        # Update the plot with new global map and trajectory
        global_map_item.setData(global_map[:, 0], global_map[:, 1])
        raw_traj_item.setData([pos[0] for pos in raw_trajectory], [pos[1] for pos in raw_trajectory])
        opt_traj_item.setData([pos[0] for pos in optimized_trajectory], [pos[1] for pos in optimized_trajectory])
        
        # Detect loop closure
        loop_index = detect_loop_closure(robot_position, raw_trajectory)
        if loop_index is not None:
            transform = np.array([
                robot_position[0] - raw_trajectory[loop_index][0],
                robot_position[1] - raw_trajectory[loop_index][1],
                0.0  # Assuming theta is 0.0 for the loop closure
            ])
            constraints.append((len(raw_trajectory) - 1, loop_index, transform))

        # Perform global optimization at specified intervals
        if len(raw_trajectory) % global_optimization_frequency == 0:
            optimized_trajectory = optimize_pose_graph(raw_trajectory, constraints)
            
            # Update the global map using the optimized poses
            global_map = update_scan_with_pose_transformation(global_map, scan_pose_pairs, optimized_trajectory, raw_trajectory)
           
            # Clear constraints after optimization
            constraints = []
            print("Pose graph optimized and global map updated.")

        # Update the application events
        app.processEvents()
        QApplication.processEvents()

print("Mapping complete.")
QApplication.exec_()