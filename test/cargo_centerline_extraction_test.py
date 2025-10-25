import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from pathlib import Path

ws_path = str(Path(__file__).resolve().parent.parent)
print("ws_path:", ws_path)


def load_point_cloud(filename) -> np.ndarray:
    """
    Load point cloud from PCD file

    Args:
        filename: input file path

    Returns:
        numpy.ndarray: Array of 3D points
    """
    try:
        pcd = o3d.io.read_point_cloud(filename)
        if not pcd.has_points():
            print(f"Loaded point cloud from {filename} has no points")
            return None
        print(f"Loaded {len(pcd.points)} points from {filename}")
        return np.asarray(pcd.points)
    except Exception as e:
        print(f"Failed to load point cloud from {filename}: {str(e)}")
        return None


# Create point cloud visualization function
def plot_point_cloud(ax, points, title, color_map="viridis", colors=None):
    """Plot point cloud on given axes"""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # Create scatter plot
    if colors is None:
        # Default coloring by Z-value
        scatter = ax.scatter(x, y, z, c=z, cmap=color_map, s=10, alpha=0.7)  # Use Z-value for coloring  # Point size  # Transparency
    else:
        # Use provided colors
        scatter = ax.scatter(x, y, z, c=colors, s=10, alpha=0.7)  # Point size  # Transparency

    # Set axis labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Set title
    ax.set_title(title, fontsize=12)

    # Set viewing angle
    ax.view_init(elev=30, azim=45)

    return scatter


# 生成示例点云数据
def generate_point_cloud(num_points=1000, scale=10):
    """生成随机点云数据"""
    points = np.random.randn(num_points, 3) * scale
    return points


# 模拟点云处理函数（实际应用中替换为您的处理逻辑）
def process_point_cloud_1(points):
    """处理点云示例1：添加噪声"""
    noise = np.random.normal(0, 0.5, points.shape)
    return points + noise


def process_point_cloud_2(points):
    """处理点云示例2：旋转点云"""
    theta = np.pi / 4  # 45度
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    return points @ rotation_matrix


def process_point_cloud_3(points):
    """处理点云示例3：下采样"""
    indices = np.random.choice(len(points), len(points) // 2, replace=False)
    return points[indices]


if __name__ == "__main__":
    import sys
    import matplotlib.gridspec as gridspec

    plt.figure(figsize=(14, 12))
    gs = gridspec.GridSpec(2, 2, width_ratios=[1, 1], height_ratios=[1, 1])

    # 创建四个3D子图
    ax1 = plt.subplot(gs[0, 0], projection="3d")
    ax2 = plt.subplot(gs[0, 1], projection="3d")
    ax3 = plt.subplot(gs[1, 0], projection="3d")
    ax4 = plt.subplot(gs[1, 1], projection="3d")

    original_points = generate_point_cloud(2000, 8)
    processed_1 = process_point_cloud_1(original_points)
    processed_2 = process_point_cloud_2(original_points)
    processed_3 = process_point_cloud_3(original_points)

    # Plot raw point cloud
    scatter1 = plot_point_cloud(ax1, original_points, "Raw Point Cloud", "viridis")

    # Plot processed point clouds
    scatter2 = plot_point_cloud(ax2, processed_1, "Processed1 (Noise Added)", "plasma")
    scatter3 = plot_point_cloud(ax3, processed_2, "Processed2 (45° Rotation)", "inferno")
    scatter4 = plot_point_cloud(ax4, processed_3, "Processed3 (50% Downsampled)", "magma")

    # Add main title
    plt.suptitle("Point Cloud Processing Results (2x2 Layout)", fontsize=16, y=0.95)
    # 添加统一的颜色条
    cbar_ax = plt.gcf().add_axes([0.92, 0.15, 0.02, 0.7])  # [left, bottom, width, height]
    cbar = plt.colorbar(scatter1, cax=cbar_ax)
    cbar.set_label("Z value", fontsize=12)

    # 调整布局
    # plt.tight_layout(rect=[0, 0, 0.9, 0.95])

    # 显示图形
    plt.show()
    plt.savefig("point_clouds_2x2.png")

    sys.exit(0)
