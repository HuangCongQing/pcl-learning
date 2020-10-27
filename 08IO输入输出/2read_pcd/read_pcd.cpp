/*
 * @Description: 读取pcd文件：https://www.cnblogs.com/li-yao7758258/p/6435568.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-09 12:56:02
 * @LastEditTime: 2020-10-27 16:27:55
 * @FilePath: /pcl-learning/08IO输入输出/2read_pcd/read_pcd.cpp
 */
#include <iostream>          //标准C++库中的输入输出的头文件
#include <pcl/io/pcd_io.h>   //PCD读写类相关的头文件
#include <pcl/point_types.h> //PCL中支持的点类型的头文件

int main(int argc, char **argv)
{
    //创建一个PointCloud<pcl::PointXYZ>    boost共享指针并进行实例化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../test_pcd.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    //默认就是而二进制块读取转换为模块化的PointCLoud格式里pcl::PointXYZ作为点类型  然后打印出来
    std::cout << "Loaded "
              << cloud->width * cloud->height // 宽*高
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " " << cloud->points[i].y
                  << " " << cloud->points[i].z << std::endl;

    return (0);
}