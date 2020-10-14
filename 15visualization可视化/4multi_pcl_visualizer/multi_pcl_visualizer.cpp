/*
 * @Description: add: 两个窗口显示点云（滤波前&滤波后） https://blog.csdn.net/weixin_45377028/article/details/104564467
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-14 11:11:11
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-14 11:40:53
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 填入点云数据
    pcl::PCDReader reader;
    // 把路径改为自己存放文件的路径
    reader.read<pcl::PointXYZ>("../table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::visualization::PCLVisualizer viewer("双窗口学习");
    //添加坐标系
    //viewer.addCoordinateSystem(0,0);

    int v1(0); //设置左右窗口
    int v2(1);

    viewer.createViewPort(0.0, 0.0, 0.5, 1, v1); //(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer.setBackgroundColor(0, 0, 0, v1);

    viewer.createViewPort(0.5, 0.0, 1, 1, v2); //(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_blue(cloud, 0, 0, 255); // 显示蓝色点云
    viewer.addPointCloud(cloud, cloud_out_blue, "cloud_out1", v1);

    pcl::PassThrough<pcl::PointXYZ> pass; //创建滤波器 pass
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    /*pcl::PCDWriter writer;*/
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_orage(cloud_filtered, 250, 128, 10); //显示橘色点云
    viewer.addPointCloud(cloud_filtered, cloud_out_orage, "cloud_out2", v2);
    //viewer.setSize(960, 780);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
