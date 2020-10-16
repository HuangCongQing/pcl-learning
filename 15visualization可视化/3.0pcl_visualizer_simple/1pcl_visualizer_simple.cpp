/*
 * @Description: Visualizer:点云可视化  参考：https://zhuanlan.zhihu.com/p/103415106
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-12 16:23:42
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-16 11:17:47
 */
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h> //PCL中支持的点类型的头文件

typedef pcl::PointXYZ PointT;

int main(int argc, char **argv)
{
    // 1 Visualizer初始化
    pcl::visualization::PCLVisualizer viewer;
    // 设置背景颜色 (0,0,0)小黑 (255,255,255)大白
    viewer.setBackgroundColor(0, 0, 0);
    //添加坐标系（即红绿蓝三色轴，放置在原点）
    viewer.addCoordinateSystem(3.0); //3.0指轴的长度
    //viewer.addCoordinateSystem (3.0,1,2,3);一个重载函数，3.0指轴的长度，放置在（1，2，3）位置
    //初始化默认相机参数
    viewer.initCameraParameters();

    // 2 向Visualize中添加点云
    //创建一个点云（或者用你的任何点云代替pc）
    // 创建点云渲染对象，导入待渲染文件
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>); //声明cloud
    pcl::io::loadPCDFile("../milk.pcd", *cloud);                                           //加载点云文件


    // 创建一个点云的句柄
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, 255, 0, 0);
    // 将点云加入到viewer
    viewer.addPointCloud<PointT>(cloud, rgb, "sample cloud33");
    //设置点云的可视化信息——这里设置了点云的大小为1.
    //注意，这里的第三个参数务必和上一段代码相同。当然，你可以在这里为多个点云设置不同的参数
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud33"); // [setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <label_pc>
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}