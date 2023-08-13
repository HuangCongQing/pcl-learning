/*
 * @Description: 文档：https://www.yuque.com/huangzhongqing/pcl/gy673qo6rl4gpxta
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2023-08-13 11:46:58
 * @LastEditTime: 2023-08-13 17:58:57
 * @FilePath: /pcl-learning/15visualization可视化/PCLVisualizer/02vis_continuous_frame/vis_continuous_frame.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h> // pcd
#include <pcl/io/ply_io.h> //ply
#include <pcl/visualization/pcl_visualizer.h>  // vis
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/filesystem.hpp> // 文件排序

int main()
{
    // 创建PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

    // 1 处理文件==================================================
    // PCD/PLY 文件夹路径
    // std::string pc_dir = "/home/hcq/github/pcl-learning/data/vis_continuous_frame";
    std::string pc_dir = "/home/hcq/github/pcl-learning/data/train_test";
    

    // 对文件列表进行排序
    // 创建一个vector来存储目录中的文件列表
    std::vector<boost::filesystem::path> file_list;
    // 遍历目录并将文件添加到file_list中
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(pc_dir); itr != end_itr; ++itr)
    {
        if (boost::filesystem::is_regular_file(itr->status()))
        {
            file_list.push_back(itr->path());
        }
    }
    // 对file_list进行排序
    std::sort(file_list.begin(), file_list.end());


    // 2 循环读取和可视化多帧点云==================================================
    // for (int frame = 0; frame < num_frames; frame++)
    for (const auto& file : file_list)
    {   
        std::cout<<file<<std::endl;
        // 创建点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 读取当前帧的点云数据
        std::string file_extension = boost::filesystem::extension(file);
        if (boost::iequals(file_extension, ".pcd")){
            pcl::io::loadPCDFile<pcl::PointXYZ>(file.string(), *cloud);
        }
        else if (boost::iequals(file_extension, ".ply"))
        {
            pcl::io::loadPLYFile<pcl::PointXYZ>(file.string(), *cloud);
        }
        else
        {
            std::cout<<"跳过不支持的文件类型: " << file.string()<<std::endl;
            continue;
        }
        
        // 设置点云颜色为红色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);

        // 添加当前帧点云数据到可视化窗口，并设置唯一的标识符
        viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, file.string());

        // 设置点云渲染大小
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, file.string());

        // 设置可视化窗口背景颜色为黑色
        viewer->setBackgroundColor(0.0, 0.0, 0.0);

        // 每帧间隔300ms，并在可视化结束后清除点云数据
        viewer->spinOnce(300);
        viewer->removePointCloud(file.string());// 可视化结束后清除点云数据
    }

    // 等待直到可视化窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    return 0;
}
