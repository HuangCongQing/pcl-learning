/*
 * @Description: 文档：https://www.yuque.com/huangzhongqing/pcl/gy673qo6rl4gpxta
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2023-08-04 01:55:30
 * @LastEditTime: 2023-08-13 13:14:04
 * @FilePath: /pcl-learning/15visualization可视化/PCLVisualizer/01vis_single_frame/01vis_single_frame.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // 创建PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../table_scene_mug_stereo_textured.pcd", *cloud);
    // pcl::io::loadPLYFile<pcl::PointXYZ>("point_cloud.pcd", *cloud);

    // 设置点云颜色为红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
  
    // 添加点云数据到可视化窗口，并设置标识符为"cloud"
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud");

    // 设置可视化窗口背景颜色为白色(1.0)  黑色（0.0）
    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    // 设置点云渲染大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    // 等待直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    return 0;
}
