/*
 * @Description: https://www.cnblogs.com/li-yao7758258/p/6437440.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-08 21:46:40
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-08 22:03:47
 */
#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  srand (time (NULL));   //用系统时间初始化随机种子
  //创建一个PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // 随机点云生成
  cloud->width = 1000;             //此处点云数量
  cloud->height = 1;                //表示点云为无序点云
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)   //循环填充点云数据
  {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
 //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
 //设置搜索空间
  kdtree.setInputCloud (cloud);
  //设置查询点并赋随机值
  pcl::PointXYZ searchPoint;
  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // K 临近搜索
  //创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
  std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
  //打印相关信息
  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )  //执行K近邻搜索
  {
     //打印所有近邻坐标
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }
  /**********************************************************************************
   下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
   pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
   ********************************************************************************/
  // 半径 R内近邻搜索方法

  std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
  std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);   //随机的生成某一半径
  //打印输出
  std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )  //执行半径R内近邻搜索方法
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}

// K nearest neighbor search at (496.165 566.915 291.089) with K=10
//     489.233 546.223 309.002 (squared distance: 797.095)
//     520.672 589.71 262.249 (squared distance: 1951.96)
//     463.981 513.441 291.076 (squared distance: 3895.31)
//     570.873 524.83 291.856 (squared distance: 7352.9)
//     446.205 583.773 359.421 (squared distance: 7449.44)
//     563.249 506.541 261.833 (squared distance: 9001.09)
//     419.788 605.717 346.714 (squared distance: 10433.2)
//     491.057 669.789 306.901 (squared distance: 10859.2)
//     388.539 556.211 300.09 (squared distance: 11779.1)
//     518.13 555.791 397.364 (squared distance: 11900.5)
// Neighbors within radius search at (496.165 566.915 291.089) with radius=6.36844
