/*
 * @Description: 法向量估计(运行耗时2min)。：https://www.cnblogs.com/li-yao7758258/p/6479255.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-19 16:33:43
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-10-19 14:34:00
 * @FilePath: /pcl-learning/10features特征/1法向量估计/normal_estimation.cpp
 */
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

int
main ()
 {
//打开点云代码
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile ("../table_scene_lms400.pcd", *cloud);

//创建法线估计估计向量
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud(cloud);
//创建一个空的KdTree对象，并把它传递给法线估计向量
//基于给出的输入数据集，KdTree将被建立
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
ne.setSearchMethod(tree);
//存储输出数据
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//使用半径在查询点周围3厘米范围内的所有临近元素
ne.setRadiusSearch(0.03);
//计算特征值
ne.compute(*cloud_normals);
// cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同的尺寸

// 存储特征值为点云
pcl::PCDWriter writer;
writer.write<pcl::Normal> ( "../table_cloud_normals.pcd" , *cloud_normals, false); // 保存文件
//可视化
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0.0, 0.0, 0.0);
viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals); // 将估计的点云表面法线添加到屏幕。

while (!viewer.wasStopped ())
{
     viewer.spinOnce ();
}

return 0;
}