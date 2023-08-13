/*
 * @Description: 文档：https://www.yuque.com/huangzhongqing/pcl/gy673qo6rl4gpxta
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2023-08-13 11:46:58
 * @LastEditTime: 2023-08-13 13:15:25
 * @FilePath: /pcl-learning/15visualization可视化/PCLVisualizer/03vis_continuous_frame_by_mouse/vis_continuous_frame_by_mouse.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>

bool continue_visualization = true;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
    // if (event.getKeyCode() != 'c' && event.keyDown())
    if (event.getKeyCode() == 's' && event.keyDown())
    {
        // continue_visualization = !continue_visualization; // 切换可视化状态
        continue_visualization = false; // 切换可视化状态
    }
    else if (event.getKeyCode() == 'c' && event.keyDown())
    {
        continue_visualization = true; // 按下's'键，继续可视化
    }
}

int main()
{
    // 创建PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

    // 注册键盘事件回调函数
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    // 循环读取和可视化多帧点云
    for (int frame = 0; frame < num_frames; frame++)
    {
        // 创建点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 读取当前帧的点云数据
        pcl::io::loadPCDFile<pcl::PointXYZ>("frame_" + std::to_string(frame) + ".pcd", *cloud);
        // pcl::io::loadPLYFile<pcl::PointXYZ>("frame_" + std::to_string(frame) + ".pcd", *cloud);

        // 添加当前帧点云数据到可视化窗口，并设置唯一的标识符
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud_" + std::to_string(frame));

        // 设置点云渲染大小
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_" + std::to_string(frame));

        // 设置可视化窗口背景颜色为白色
        viewer->setBackgroundColor(1.0, 1.0, 1.0);
        
        // 更新文本显示状态
        std::string status_text = continue_visualization ? "Visualization Status: Running" : "Visualization Status: Paused";
        text_viewer->updateText(status_text, 10, 10, "status_text");

        // 每帧可视化后等待按键事件(修改continue_visualization的值)，以及每帧间隔300ms
        while (continue_visualization)
        {
            viewer->spinOnce(300);  // 每帧间隔300ms
            boost::this_thread::sleep_for(boost::chrono::milliseconds(300)); // 等待300ms
        }
        viewer->removePointCloud("cloud_" + std::to_string(frame));// 可视化结束后清除点云数据
    }

    return 0;
}
