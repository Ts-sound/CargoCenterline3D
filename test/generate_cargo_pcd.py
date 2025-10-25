"""
generate_cargo_pointcloud.py - 生成3D货箱点云数据工具

用法:
  python generate_cargo_pointcloud.py [选项]

选项:
  --length LENGTH      货箱长度 (默认: 3.0)
  --width WIDTH        货箱宽度 (默认: 2.0)
  --height HEIGHT      货箱高度 (默认: 1.5)
  --num_points POINTS  总点数 (默认: 10000)
  --noise NOISE        噪声水平 (默认: 0.01)
  --output OUTPUT      输出PCD文件名 (默认: cargo_box.pcd)
  --top                是否包括顶面点云 (默认: False)
  --thickness THICK    箱壁厚度 (默认: 0.05)
  --holes              是否在箱壁上生成孔洞
  --visualize          可视化生成的点云

示例:
  python generate_cargo_pointcloud.py --length 4.0 --width 2.5 --height 2.0 --num_points 20000 --output large_cargo.pcd
"""

import argparse
import numpy as np
import open3d as o3d
import random
from pathlib import Path

ws_path = str(Path(__file__).resolve().parent.parent)
print("ws_path:", ws_path)


def generate_cargo_box_points(
    length=3.0,
    width=2.0,
    height=1.5,
    num_points=10000,
    noise=0.01,
    include_top=False,
    wall_thickness=0.05,
):
    """
    生成带噪声的货箱点云数据

    参数:
        length: 货箱长度 (x轴)
        width: 货箱宽度 (y轴)
        height: 货箱高度 (z轴)
        num_points: 总点数
        noise: 噪声水平
        include_top: 是否包括顶面点云
        wall_thickness: 箱壁厚度

    返回:
        numpy数组形式的点云数据
    """
    points = []

    # 计算各面的点数分配
    base_points = int(num_points * 0.4)
    side_points = int(num_points * 0.4)
    end_points = int(num_points * 0.15)
    top_points = int(num_points * 0.05) if include_top else 0

    # 底面点云 (主平面)
    bottom_points = np.random.rand(base_points, 3)
    bottom_points[:, 0] = bottom_points[:, 0] * length
    bottom_points[:, 1] = bottom_points[:, 1] * width
    bottom_points[:, 2] = 0 + np.random.normal(0, noise, base_points)
    points.append(bottom_points)

    # 侧面点云 (左右两侧)
    for _ in range(side_points // 2):
        # 随机选择左侧或右侧
        side = random.choice([0, width])

        # 随机位置
        x = random.uniform(0, length)
        z = random.uniform(0, height)

        # 添加厚度
        y = side + random.uniform(0, wall_thickness) * (-1 if side == width else 1)

        # 添加噪声
        x += np.random.normal(0, noise)
        y += np.random.normal(0, noise)
        z += np.random.normal(0, noise)

        points.append(np.array([[x, y, z]]))

    # 端面点云 (前后两端)
    for _ in range(end_points):
        # 随机选择前端或后端
        end = random.choice([0, length])

        # 随机位置
        y = random.uniform(0, width)
        z = random.uniform(0, height)

        # 添加厚度
        x = end + random.uniform(0, wall_thickness) * (-1 if end == length else 1)

        # 添加噪声
        x += np.random.normal(0, noise)
        y += np.random.normal(0, noise)
        z += np.random.normal(0, noise)

        points.append(np.array([[x, y, z]]))

    # 顶面点云 (可选)
    if include_top:
        top = np.random.rand(top_points, 3)
        top[:, 0] = top[:, 0] * length
        top[:, 1] = top[:, 1] * width
        top[:, 2] = height + np.random.normal(0, noise, top_points)
        points.append(top)

    # 合并所有点
    all_points = np.vstack(points)

    # 确保点数正确
    if len(all_points) > num_points:
        all_points = all_points[:num_points]

    return all_points


def save_to_pcd(points, filename):
    """
    将点云保存为PCD文件

    参数:
        points: numpy数组形式的点云数据
        filename: 输出文件名
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Saved point cloud to {filename} ({len(points)} points)")


def main():
    """主函数：解析参数并生成点云"""
    parser = argparse.ArgumentParser(description="生成3D货箱点云数据")
    parser.add_argument("--length", type=float, default=3.0, help="货箱长度 (默认: 3.0)")
    parser.add_argument("--width", type=float, default=2.0, help="货箱宽度 (默认: 2.0)")
    parser.add_argument("--height", type=float, default=1.5, help="货箱高度 (默认: 1.5)")
    parser.add_argument("--num_points", type=int, default=10000, help="总点数 (默认: 10000)")
    parser.add_argument("--noise", type=float, default=0.01, help="噪声水平 (默认: 0.01)")
    parser.add_argument("--output", type=str, default="cargo_box.pcd", help="")
    parser.add_argument("--top", action="store_true", help="包括顶面点云 (默认: False)")
    parser.add_argument("--thickness", type=float, default=0.05, help="箱壁厚度 (默认: 0.05)")

    args = parser.parse_args()

    print("生成货箱点云数据...")
    print(f"尺寸: {args.length} x {args.width} x {args.height}")
    print(f"点数: {args.num_points}, 噪声: {args.noise}")
    print(f"箱壁厚度: {args.thickness},  顶面: {'是' if args.top else '否'}")

    # 生成点云
    points = generate_cargo_box_points(
        length=args.length,
        width=args.width,
        height=args.height,
        num_points=args.num_points,
        noise=args.noise,
        include_top=args.top,
        wall_thickness=args.thickness,
    )

    # 保存为PCD文件
    save_to_pcd(points, ws_path + "/data/" + args.output)


if __name__ == "__main__":
    main()
