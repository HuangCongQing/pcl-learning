/*
 * @Description: 在PCL 中使用直通滤波器对点云进行滤波处理  https://www.cnblogs.com/li-yao7758258/p/6445831.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 10:36:02
 * @LastEditTime: 2020-10-20 10:45:57
 * @FilePath: /pcl-learning/09filters滤波/1直通滤波器进行滤波处理/passthrough.cpp
 */
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //生成并填充点云
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)   //填充数据
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;   //打印
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
  /************************************************************************************
   创建直通滤波器的对象，设立参数，滤波字段名被设置为Z轴方向，可接受的范围为（0.0，1.0）
   即将点云中所有点的Z轴坐标不在该范围内的点过滤掉或保留，这里是过滤掉，由函数setFilterLimitsNegative设定
   ***********************************************************************************/
  // 设置滤波器对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);            //设置输入点云
  pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (0.0, 1.0);        //设置在过滤字段的范围
  //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter (*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered

  std::cerr << "Cloud after filtering: " << std::endl;   //打印
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

  return (0);
}