/*
 * @Description: 写入数据
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-09 10:42:31
 * @LastEditTime: 2020-10-27 16:24:56
 * @FilePath: /pcl-learning/08IO输入输出/1write_pcd/write_pcd.cpp
 */
#include <iostream>          //标准C++库中的输入输出的头文件
#include <pcl/io/pcd_io.h>   //PCD读写类相关的头文件
#include <pcl/point_types.h> //PCL中支持的点类型的头文件

int main(int argc, char **argv)
{
    //实例化的模板类PointCloud  每一个点的类型都设置为pcl::PointXYZ
    /*************************************************
 点PointXYZ类型对应的数据结构
    Structure PointXYZ{
     float x;
     float y;
     float z;
    };
**************************************************/
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 创建点云  并设置适当的参数（width height is_dense）
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;                          //不是稠密型的
    cloud.points.resize(cloud.width * cloud.height); //点云总数大小
                                                     //用随机数的值填充PointCloud点云对象
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    //把PointCloud对象数据存储在 test_pcd.pcd文件中
    pcl::io::savePCDFileASCII("../test_pcd.pcd", cloud);

    //打印输出存储的点云数据
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size(); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}