/*
 * @Description: 双窗口显示 https://blog.csdn.net/uranus1992/article/details/84545820
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-14 11:11:11
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-14 11:18:43
 */

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

viewer->initCameraParameters()
// V1视口位于窗口左半部分，V2视口位于右半部分。
int v1(0);

viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);

viewer->setBackgroundColor(0, 0, 0, v1);

viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);

pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
int v2(0);

viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2); // createViewPort是用于创建新视口的函数，所需的4个参数分别是视口在X轴的最小值、最大值，Y轴的最小值、最大值，取值在0-1.

viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);

viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);

viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
