/*
 * @Description: 实现圆柱体模型的分割：采用随机采样一致性估计从带有噪声的点云中提取一个圆柱体模型。：https://www.cnblogs.com/li-yao7758258/p/6496664.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-13 16:33:43
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-14 18:41:30
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h> // 可视化

typedef pcl::PointXYZ PointT;

int main(int argc, char **argv)
{
    // All the objects needed
    pcl::PCDReader reader;                                    //PCD文件读取对象
    pcl::PassThrough<PointT> pass;                            //直通滤波对象
    pcl::NormalEstimation<PointT, pcl::Normal> ne;            //法线估计对象
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; //分割对象
    pcl::PCDWriter writer;                                    //PCD文件读取对象
    pcl::ExtractIndices<PointT> extract;                      //点提取对象
    pcl::ExtractIndices<pcl::Normal> extract_normals;         ///点提取对象
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Read in the cloud data
    reader.read("../table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    // 直通滤波，将Z轴不在（0，1.5）范围的点过滤掉，将剩余的点存储到cloud_filtered对象中
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    //获取平面模型的系数和处在平面的内点
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // 从点云中抽取分割的处在平面上的点集
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // 存储分割得到的平面上的点到点云文件
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
    writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false); // 分割得到的平面

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);        //设置对估计模型优化
    seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱形
    seg.setMethodType(pcl::SAC_RANSAC);       //参数估计方法
    seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
    seg.setMaxIterations(10000);              //设置迭代的最大次数10000
    seg.setDistanceThreshold(0.05);           //设置内点到模型的距离允许最大值
    seg.setRadiusLimits(0, 0.1);              //设置估计出的圆柱模型的半径的范围
    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
        writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false); // 分割得到的平面
    }
    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("three 三窗口 "));
    int v1(0); //设置左右窗口
    int v2(1);
    int v3(2);
    viewer->createViewPort(0.0, 0.0, 0.5,1.0, v1); //(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2);
    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v3);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_red(cloud, 255, 0, 0); // 显示红色点云
    viewer->addPointCloud(cloud, cloud_out_red, "cloud_out1", v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_green(cloud, 0, 255, 0); // 显示绿色点云
    viewer->addPointCloud(cloud_plane, cloud_out_green, "cloud_out2", v2);  // 平面

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_blue(cloud, 0, 0, 255); // 显示蓝色点云
    viewer->addPointCloud(cloud_cylinder, cloud_out_blue, "cloud_out3", v3); // 圆柱

    // 1. 阻塞式
    viewer->spin();
    // 2. 非阻塞式
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce();
    // }
    return (0);
}