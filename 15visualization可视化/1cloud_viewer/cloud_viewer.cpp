/*
 * @Description: PCL 可视化入门：https://www.cnblogs.com/li-yao7758258/p/6442156.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-12 09:59:01
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-11-02 22:05:15
 */
#include <pcl/visualization/cloud_viewer.h> //类cloud_viewer头文件申明
#include <iostream>                         //标准输入输出头文件申明
#include <pcl/io/io.h>                      //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                  //PCD文件读取

/**********************************************************************************
  函数是作为回调函数，在主函数中只注册一次 ，函数实现对可视化对象背景颜色的设置，添加一个圆球几何体
*********************************************************************************/
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0); //设置背景颜色
    pcl::PointXYZ o;                          //存储球的圆心位置
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0); //添加圆球几何对象
    std::cout << "i only run once" << std::endl;
}
/***********************************************************************************
   作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串
   *************************************************************************************/
void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0); // 添加文字

    //FIXME: possible race condition here:
    user_data++;
}
/**************************************************************
  首先加载点云文件到点云对象，并初始化可视化对象viewer，注册上面的回
   调函数，执行循环直到收到关闭viewer的消息退出程序
   *************************************************************/
int main()
{
    // 创建点云渲染对象，导入待渲染文件
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); //声明cloud
    pcl::io::loadPCDFile("../room_scan1.pcd", *cloud);                                        //加载点云文件

    //创建点云渲染句柄
    pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
    //showCloud函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);

    //下面的内容是一个模板，支持用户进行更复杂的操作。

    //该注册函数在可视化的时候只执行一次
    viewer.runOnVisualizationThreadOnce(viewerOneOff); //只运行一次的业务逻辑可以放在viewerOneOff函数里，比如设置背景、画个三维球等等。

    //该注册函数在渲染输出时每次都调用
    viewer.runOnVisualizationThread(viewerPsycho); //需要每轮渲染的业务逻辑可以放在viewerPsycho
    //现在的业务逻辑仅仅是完成用户数据的单调增加，此处还可以完成更多丰富的操作。

    while (!viewer.wasStopped())
    {
        //此处可以添加其他处理
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }
    return 0;
}
//  总结

// pcl_points_visualization.cpp: 定义控制台应用程序的入口点。

// #include "stdafx.h"
// #include<pcl/visualization/cloud_viewer.h>
// #include<pcl/point_cloud.h>
// #include<pcl/io/pcd_io.h>

// int main()
// {
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);
// 	pcl::visualization::CloudViewer viewer("cloud viewer");
// 	viewer.showCloud(cloud);
// 	while (!viewer.wasStopped())
// 	{
// 	}
//     return 0;
// }

