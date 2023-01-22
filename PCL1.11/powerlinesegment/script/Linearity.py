'''
Description: 
Author: HCQ
Company(School): UCAS
Email: 1756260160@qq.com
Date: 2023-01-14 00:13:24
LastEditTime: 2023-01-14 01:06:59
FilePath: /electrical-wire-around-obstacle-detection/powerlinesegment/script/Linearity.py
'''
import open3d as o3d
import numpy as np


def pass_through(cloud, limit_min=1, limit_max=3):
    """
    高程滤波
    :param cloud:输入点云
    :param limit_min: 高程最小值
    :param limit_max: 高程最大值
    :return: 高程较低点，高程较高点
    """
    points = np.asarray(cloud.points)
    ind = np.where((points[:, 2] >= limit_min) & (points[:, 2] <= limit_max))[0]
    low_cloud = cloud.select_by_index(ind) # 范围内点的下标（低点）
    high_cloud = cloud.select_by_index(ind, invert=True)
    return low_cloud, high_cloud


def pca_compute(data, sort=True):
    """
     SVD分解计算点云的特征值
    :param data: 输入数据
    :param sort: 是否将特征值进行排序
    :return: 特征值
    """
    average_data = np.mean(data, axis=0)  # 求均值
    decentration_matrix = data - average_data  # 去中心化
    H = np.dot(decentration_matrix.T, decentration_matrix)  # 求解协方差矩阵 H
    _, eigenvalues, eigenvectors_T = np.linalg.svd(H)  # SVD求解特征值

    if sort:
        sort = eigenvalues.argsort()[::-1]  # 降序排列
        eigenvalues = eigenvalues[sort]  # 索引
    return eigenvalues

# main
def power_line_segmentation(power_line_cloud, threshold=0.81):
    """
    计算每一个点的线性特征，并根据线性特征提取线点云
    :param power_line_cloud: 输入点云
    :param threshold: 线特征阈值
    :return: 线点云和线之外的点云
    """
    # low, high = pass_through(power_line_cloud, 0, 20)
    low, high = pass_through(power_line_cloud, -100, -20) # 参数修改

    points = np.asarray(high.points)
    # 计算每个点的特征值
    radius = 1.5  # 近邻点搜索的范围
    kdtree = o3d.geometry.KDTreeFlann(high)
    num_points = len(high.points)
    linear = []  # 储存线性特征
    for i in range(num_points):
        k, idx, _ = kdtree.search_radius_vector_3d(high.points[i], radius)

        neighbors = points[idx, :]
        w = pca_compute(neighbors)  # w为特征值

        l1 = w[0]  # 点云的特征值lamda1
        l2 = w[1]  # 点云的特征值lamda2
        L = np.divide((l1 - l2), l1, out=np.zeros_like((l1 - l2)), where=l1 != 0)
        linear.append(L)

    linear = np.array(linear)
    # 设置阈值提取电线
    idx = np.where(linear > threshold)[0]
    line_cloud_ = high.select_by_index(idx)  # 提取点线上的点
    out_line_cloud_ = high.select_by_index(idx, invert=True) + low  # 提取电线之外的点

    return line_cloud_, out_line_cloud_


if __name__ == '__main__':
    # --------------------读取点云---------------------
    point_cloud = o3d.io.read_point_cloud("dx.pcd")
    # --------------------分割点云---------------------
    line_cloud, out_line_cloud = power_line_segmentation(point_cloud,
                                                         threshold=0.81)
    # --------------------保存点云---------------------
    o3d.io.write_point_cloud("电线部分.pcd", line_cloud)
    o3d.io.write_point_cloud("非电线部分.pcd", out_line_cloud)
    # ------------------可视化分割结果-----------------
    line_cloud.paint_uniform_color([1, 0, 0])
    out_line_cloud.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([line_cloud, out_line_cloud])