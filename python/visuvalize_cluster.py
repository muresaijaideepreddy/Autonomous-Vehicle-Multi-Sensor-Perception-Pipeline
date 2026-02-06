import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# Load data
data = np.loadtxt("clusters.xyz")

points = data[:, :3]
cluster_ids = data[:, 3].astype(int)

# Normalize cluster IDs for coloring
unique_clusters = np.unique(cluster_ids)
colors = plt.get_cmap("tab20")(cluster_ids % 20)[:, :3]

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualize
o3d.visualization.draw_geometries(
    [pcd],
    window_name="LiDAR Clusters",
    width=1280,
    height=720,
    point_show_normal=False
)
