import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull
import os, logging
import random

logging.basicConfig(
    level=logging.WARN,
    format="[%(levelname)s|%(filename)s] [%(funcName)s:%(lineno)d] %(message)s",
)


def detect_plane_open3d(points, distance_threshold=0.02, ransac_n=3, num_iterations=1000) -> tuple:
    """
    Detect the dominant plane using Open3D's RANSAC implementation

    Args:
        points: numpy array of 3D points
        distance_threshold: Max distance for point to be considered inlier
        ransac_n: Number of points to sample for plane fitting
        num_iterations: Number of RANSAC iterations

    Returns:
        tuple: (plane_equation, inlier_indices)
    """
    if points is None or len(points) < ransac_n:
        logging.error("Not enough points for plane detection")
        return None, None

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    try:
        # Detect plane using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
        logging.info(f"Detected plane with {len(inliers)} inliers")
        return plane_model, inliers
    except Exception as e:
        logging.error(f"Plane detection failed: {str(e)}")
        return None, None


def project_points_to_plane(points, plane_equation):
    """
    Project 3D points onto a plane

    Args:
        points: numpy array of 3D points (Nx3)
        plane_equation: [a, b, c, d] for plane equation ax+by+cz+d=0

    Returns:
        numpy.ndarray: Projected points on the plane (Nx3)
    """
    if points is None or plane_equation is None:
        return None

    a, b, c, d = plane_equation
    normal = np.array([a, b, c])
    normal_norm = np.linalg.norm(normal)

    # Normalize plane equation
    if normal_norm > 0:
        normal = normal / normal_norm
        d = d / normal_norm

    # Project each point onto the plane
    projected = []
    for point in points:
        # Distance from point to plane
        dist = np.dot(normal, point) + d

        # Projection formula: P' = P - dist * normal
        proj_point = point - dist * normal
        projected.append(proj_point)

    return np.array(projected)


def compute_centerline(projected_points):
    """
    Compute symmetry centerline from projected points

    Args:
        projected_points: Points projected onto the plane (Nx3)

    Returns:
        tuple: (centerline_points, left_boundary, right_boundary)
    """
    if projected_points is None or len(projected_points) < 3:
        logging.error("Not enough points for centerline computation")
        return None, None, None

    # Convert to 2D by ignoring z-coordinate (since we're on a plane)
    points_2d = projected_points[:, :2]

    try:
        # Compute convex hull to get boundary points
        hull = ConvexHull(points_2d)
        boundary_indices = hull.vertices
        boundary_points = points_2d[boundary_indices]

        # Sort boundary points by x-coordinate
        sorted_indices = np.argsort(boundary_points[:, 0])
        sorted_boundary = boundary_points[sorted_indices]

        # Find midpoint in y-direction to separate left and right boundaries
        mid_y = np.mean(sorted_boundary[:, 1])

        # Separate left and right boundaries
        left_points = []
        right_points = []

        for point in sorted_boundary:
            if point[1] < mid_y:
                left_points.append(point)
            else:
                right_points.append(point)

        left_points = np.array(left_points)
        right_points = np.array(right_points)

        # Sort left and right points by x-coordinate
        left_points = left_points[np.argsort(left_points[:, 0])]
        right_points = right_points[np.argsort(right_points[:, 0])]

        # Interpolate to have same number of points
        min_len = min(len(left_points), len(right_points))
        left_points = left_points[:min_len]
        right_points = right_points[:min_len]

        # Compute centerline as midpoint between left and right boundaries
        centerline = (left_points + right_points) / 2

        # Add z-coordinate back (using average z from projected points)
        avg_z = np.mean(projected_points[:, 2])
        centerline = np.hstack([centerline, np.full((len(centerline), 1), avg_z)])
        left_points = np.hstack([left_points, np.full((len(left_points), 1), avg_z)])
        right_points = np.hstack([right_points, np.full((len(right_points), 1), avg_z)])

        return centerline, left_points, right_points
    except Exception as e:
        logging.error(f"Centerline computation failed: {str(e)}")
        return None, None, None


if __name__ == "__main__":
    import sys

    sys.exit(0)
