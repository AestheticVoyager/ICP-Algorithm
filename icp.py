
import numpy as np
from scipy.spatial import KDTree

def nearest_neighbors(source, target):
    """Find the nearest neighbors for each point in the source point cloud."""
    tree = KDTree(target)
    distances, indices = tree.query(source)
    return indices

def compute_transformation(source, target):
    """Compute the optimal rotation and translation to align source to target."""
    # Compute centroids
    centroid_source = np.mean(source, axis=0)
    centroid_target = np.mean(target, axis=0)

    # Center the points
    source_centered = source - centroid_source
    target_centered = target - centroid_target

    # Compute covariance matrix
    H = np.dot(source_centered.T, target_centered)

    # Singular Value Decomposition
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # Ensure a right-handed coordinate system
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Compute translation
    t = centroid_target - np.dot(R, centroid_source)

    return R, t

def apply_transformation(points, R, t):
    """Apply the rotation and translation to the points."""
    return np.dot(points, R.T) + t

def icp(source, target, max_iterations=20, tolerance=1e-6):
    """Perform the ICP algorithm to align source to target."""
    for i in range(max_iterations):
        # Step 1: Find nearest neighbors
        indices = nearest_neighbors(source, target)
        matched_target = target[indices]

        # Step 2: Compute transformation
        R, t = compute_transformation(source, matched_target)

        # Step 3: Apply transformation
        source = apply_transformation(source, R, t)

        # Check for convergence
        if np.mean(np.linalg.norm(matched_target - source, axis=1)) < tolerance:
            break

    return source, R, t

# Example usage:
if __name__ == "__main__":
    # Sample point clouds
    source_points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
    target_points = np.array([[0.5, 0.5, 0], [1.5, 0.5, 0], [0.5, 1.5, 0]])

    aligned_points, rotation, translation = icp(source_points, target_points)
    print("Aligned Points:\n", aligned_points)
    print("Rotation Matrix:\n", rotation)
    print("Translation Vector:\n", translation)
