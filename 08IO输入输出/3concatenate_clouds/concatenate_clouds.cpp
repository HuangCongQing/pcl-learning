/*
 * @Description: 点云之间的连接：https://www.cnblogs.com/li-yao7758258/p/6435759.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-09 14:20:59
 * @LastEditTime: 2020-10-27 16:30:21
 * @FilePath: /pcl-learning/08IO输入输出/3concatenate_clouds/concatenate_clouds.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>   //io模块
#include <pcl/point_types.h> //数据类型

int main(int argc, char **argv)
{
    std::cout << "argc: " << argc<<std::endl;
    std::cout << "argv: " << argv[1] << std::endl;
    if (argc != 2) //提示如果执行可执行文件输入两个参数 -f 或者-p
    {
        std::cerr << "please specify command line arg '-f' or '-p'" << std::endl;
        exit(0);
    }
    //申明三个pcl::PointXYZ点云数据类型，分别为cloud_a, cloud_b, cloud_c
    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
    //存储进行连接时需要的Normal点云,Normal (float n_x, float n_y, float n_z)
    pcl::PointCloud<pcl::Normal> n_cloud_b;
    //存储连接XYZ与normal后的点云
    pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

    // 创建点云数据
    //设置cloud_a的个数为5
    cloud_a.width = 5;
    cloud_a.height = cloud_b.height = n_cloud_b.height = 1; //设置都为无序点云
    cloud_a.points.resize(cloud_a.width * cloud_a.height);  //总数
    if (strcmp(argv[1], "-p") == 0)                         //判断是否为连接a+b=c(点云连接)
    {
        cloud_b.width = 3;
        cloud_b.points.resize(cloud_b.width * cloud_b.height);
    }
    else
    {
        n_cloud_b.width = 5; //如果是连接XYZ与normal则生成5个法线（字段间连接）
        n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
    }
    //以下循环生成无序点云填充上面定义的两种类型的点云数据
    for (size_t i = 0; i < cloud_a.points.size(); ++i)
    { //cloud_a产生三个点（每个点都有X Y Z 三个随机填充的值）
        cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    if (strcmp(argv[1], "-p") == 0)
        for (size_t i = 0; i < cloud_b.points.size(); ++i)
        { //如果连接a+b=c，则cloud_b用三个点作为xyz的数据
            cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }
    else
        for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
        { //如果连接xyz+normal=xyznormal则n_cloud_b用5个点作为normal数据
            n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
            n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
            n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
        }
    /*******************************************************************
定义了连接点云会用到的5个点云对象：3个输入（cloud_a cloud_b 和n_cloud_b）
两个输出（cloud_c  n_cloud_c）然后就是为两个输入点云cloud_a和 cloud_b或者cloud_a 和n_cloud_b填充数据  

********************************************************************/
    //输出Cloud A
    std::cerr << "Cloud A: " << std::endl;
    for (size_t i = 0; i < cloud_a.points.size(); ++i)
        std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;
    //输出Cloud B
    std::cerr << "Cloud B: " << std::endl;
    if (strcmp(argv[1], "-p") == 0)
        for (size_t i = 0; i < cloud_b.points.size(); ++i)
            std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;
    else //输出n_Cloud_b
        for (size_t i = 0; i < n_cloud_b.points.size(); ++i)
            std::cerr << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << std::endl;

    // Copy the point cloud data
    if (strcmp(argv[1], "-p") == 0)
    {
        cloud_c = cloud_a;
        cloud_c += cloud_b; //把cloud_a和cloud_b连接一起创建cloud_c  后输出
        std::cerr << "Cloud C: " << std::endl;
        for (size_t i = 0; i < cloud_c.points.size(); ++i)
            std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;
    }
    else
    { //连接字段  把cloud_a和 n_cloud_b字段连接 一起创建 p_n_cloud_c)
        pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
        std::cerr << "Cloud C: " << std::endl;
        for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i)
            std::cerr << "    " << p_n_cloud_c.points[i].x << " " << p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " << p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << std::endl;
    }
    return (0);
}

// 点云连接和字段间连接的区别，字段间连接是在行的基础后连接，而点云连接是在列的下方连接，最重要的就是要考虑维度问题，同时每个点云都有XYZ三个数据值