/*
 * @Description: 如何从点云创建深度图，  
 * https://www.cnblogs.com/li-yao7758258/p/6474699.html    
 * http://robot.czxy.com/docs/pcl/chapter02/range_image/#_1  推荐
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-21 15:08:41
 * @LastEditTime: 2020-10-21 15:52:15
 * @FilePath: /pcl-learning/06range-images深度图像/1从点云创建深度图/range_image_creation.cpp
 */

#include <pcl/range_image/range_image.h> // //关于深度图像的头文件
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>  //  //PCL可视化的头文件
#include <pcl/visualization/range_image_visualizer.h> // //深度图可视化的头文件

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &pointCloud = *pointCloudPtr;

    // 创建一个矩形形状的点云
    // Generate the data
    // for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    // {
    //     for (float z = -0.5f; z <= 0.5f; z += 0.01f)
    //     {
    //         pcl::PointXYZ point;
    //         point.x = 2.0f - y;
    //         point.y = y;
    //         point.z = z;
    //         pointCloud.points.push_back(point);
    //     }
    // }
    // pointCloud.width = (uint32_t)pointCloud.points.size();
    // pointCloud.height = 1;

    pcl::io::loadPCDFile("../bunny.pcd", pointCloud);
    // pcl::io::loadPCDFile("../table_scene_lms400_downsampled.pcd", pointCloud);

    //实现一个呈矩形形状的点云
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution   根据之前得到的点云图，通过1deg的分辨率生成深度图。
    //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
    float angularResolution = (float)(1.0f * (M_PI / 180.0f)); //  弧度1°
                                                               //max_angle_width为模拟的深度传感器的水平最大采样角度，
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));   //  弧度360°
    //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // 弧度180°
    //传感器的采集位置
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    //深度图像遵循坐标系统
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00; //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
    float minRange = 0.0f;   //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
    int borderSize = 1;      //border_size获得深度图像的边缘的宽度

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage); // 用于可视化？共享指针
    pcl::RangeImage &rangeImage = *range_image_ptr;

    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    /*
    关于range_image.createFromPointCloud（）参数的解释 （涉及的角度都为弧度为单位） ：
    point_cloud为创建深度图像所需要的点云
    angular_resolution_x深度传感器X方向的角度分辨率
    angular_resolution_y深度传感器Y方向的角度分辨率
    pcl::deg2rad (360.0f)深度传感器的水平最大采样角度
    pcl::deg2rad (180.0f)垂直最大采样角度
    scene_sensor_pose设置的模拟传感器的位姿是一个仿射变换矩阵，默认为4*4的单位矩阵变换
    coordinate_frame定义按照那种坐标系统的习惯  默认为CAMERA_FRAME
    noise_level  获取深度图像深度时，邻近点对查询点距离值的影响水平
    min_range 设置最小的获取距离，小于最小的获取距离的位置为传感器的盲区
    border_size  设置获取深度图像边缘的宽度 默认为0
    */
    std::cout << rangeImage << "\n";
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    // 添加深度图点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");

    // 添加原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_image_color_handler(pointCloudPtr, 255, 100, 0);
    viewer.addPointCloud(pointCloudPtr, org_image_color_handler, "orginal image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orginal image");

    viewer.initCameraParameters();
    viewer.addCoordinateSystem(1.0);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
    return (0);
}