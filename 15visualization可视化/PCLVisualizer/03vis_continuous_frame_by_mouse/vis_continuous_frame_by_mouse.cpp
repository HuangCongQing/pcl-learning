/*
 * @Description: 文档：https://www.yuque.com/huangzhongqing/pcl/gy673qo6rl4gpxta
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2023-08-13 11:46:58
 * @LastEditTime: 2023-08-13 20:59:49
 * @FilePath: /pcl-learning/15visualization可视化/PCLVisualizer/03vis_continuous_frame_by_mouse/vis_continuous_frame_by_mouse.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h> // pcd
#include <pcl/io/ply_io.h> //ply
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>


constexpr char kPauseKey[] = "s"; // keyboard_ ="s" 就暂停播放，按到其他键就继续播放
std::string keyboard_ = "";


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* param)
{
  if (event.keyDown()) {
    std::string* key = reinterpret_cast<std::string*>(param);
    *key = event.getKeySym(); // 
  }
}

int main()
{
    // 创建PCLVisualizer对象(初始化)
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

    // 注册键盘事件回调函数
    viewer->registerKeyboardCallback(keyboardEventOccurred, static_cast<void*>(&keyboard_));
    
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


    // 循环读取和可视化多帧点云
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
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, file.string());

        // 设置可视化窗口背景颜色为黑色
        viewer->setBackgroundColor(0.0, 0.0, 0.0);
        
        // 更新文本显示状态
        std::string status_text = keyboard_ != kPauseKey ? "Visualization Status: Running" : "Visualization Status: Paused";
        viewer->addText(status_text, 20, 60, file.string());
        viewer->updateText(status_text, 20, 60, file.string()); //TODO: 更新文字失败
        
        // 默认暂停300ms
        viewer->spinOnce(300);  // 调用内部渲染函数一次，重新渲染输出时间最大不超过time,单位ms 每帧间隔300ms
        boost::this_thread::sleep_for(boost::chrono::milliseconds(300)); // 等待300ms
        // 每帧可视化后等待按键事件(修改continue_visualization的值)，以及每帧间隔300ms// 每帧可视化后等待按键事件(keyboard_ ="s" 就暂停播放，按到其他键就继续播放)，以及每帧间隔300ms
        while (keyboard_ == kPauseKey)
        {
            viewer->spinOnce(300);  // 每帧间隔300ms
            boost::this_thread::sleep_for(boost::chrono::milliseconds(300)); // 等待300ms
        }
        // 删除之前的文本信息
        viewer->removeShape(file.string());
        // 删除点云
        viewer->removePointCloud(file.string());// 可视化结束后清除点云数据
    }

    return 0;
}
