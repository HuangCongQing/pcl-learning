/*
 * @Description: 文档：https://www.yuque.com/huangzhongqing/pcl/gy673qo6rl4gpxta
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2023-08-13 11:46:58
 * @LastEditTime: 2023-08-13 13:15:37
 * @FilePath: /pcl-learning/15visualization可视化/PCLVisualizer/02vis_continuous_frame/vis_continuous_frame.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

int main()
{
    // 创建PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

    // 循环读取和可视化多帧点云
    for (int frame = 0; frame < num_frames; frame++)
    {
        // 创建点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 读取当前帧的点云数据
        pcl::io::loadPCDFile<pcl::PointXYZ>("frame_" + std::to_string(frame) + ".pcd", *cloud);
        // pcl::io::loadPLYFile<pcl::PointXYZ>("frame_" + std::to_string(frame) + ".pcd", *cloud);

        // 添加当前帧点云数据到可视化窗口，并设置唯一的标识符
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud_" + std::to_string(frame));

        // 设置点云渲染大小
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_" + std::to_string(frame));

        // 设置可视化窗口背景颜色为白色
        viewer->setBackgroundColor(1.0, 1.0, 1.0);

        // 每帧间隔300ms，并在可视化结束后清除点云数据
        viewer->spinOnce(300);
        viewer->removePointCloud("cloud_" + std::to_string(frame));// 可视化结束后清除点云数据
    }

    // 等待直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    return 0;
}
