import open3d as o3d
import Linearity

# --------------------读取点云---------------------
point_cloud = o3d.io.read_point_cloud("51410110E0C24609B50084B5D8ACA393.pcd")
# --------------------分割点云---------------------
line_cloud, out_line_cloud = Linearity.power_line_segmentation(point_cloud,
                                                               threshold=0.81)
# --------------------保存点云---------------------
o3d.io.write_point_cloud("电线部分.pcd", line_cloud)
o3d.io.write_point_cloud("非电线部分.pcd", out_line_cloud)
# ------------------可视化分割结果-----------------
line_cloud.paint_uniform_color([1, 0, 0])
out_line_cloud.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([line_cloud, out_line_cloud])
