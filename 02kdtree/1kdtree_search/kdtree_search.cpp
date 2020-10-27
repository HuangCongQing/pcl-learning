/*
 * @Description: https://www.cnblogs.com/li-yao7758258/p/6437440.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-08 21:46:40
 * @LastEditTime: 2020-10-27 16:03:16
 * @FilePath: /pcl-learning/02kdtree/1kdtree_search/kdtree_search.cpp
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
    cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f); // // 产生数值为0-1024的浮点数
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
 //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; // pcl::KdTreeFLANN<PointT, Dist>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
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

  // 假设我们的kdtree返回了大于0个近邻。那么它将打印出在我们"searchPoint"附近的10个最近的邻居并把它们存到先前创立的向量中。
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

// K nearest neighbor search at (384.369 386.372 300.143) with K=10
//     451.114 363.553 229.213 (squared distance: 10006.7)
//     452.721 398.977 207.076 (squared distance: 13492.4)
//     276.971 343.368 327.111 (squared distance: 14111)
//     447.772 491.576 334 (squared distance: 16234.2)
//     410.276 386.502 175.288 (squared distance: 16259.9)
//     445.201 488.637 253.626 (squared distance: 16322.6)
//     257.128 435.577 321.633 (squared distance: 19073.3)
//     312.382 389.858 178.369 (squared distance: 20023)
//     439.166 412.965 167.177 (squared distance: 21390)
//     482.03 493.711 266.617 (squared distance: 22183.4)
// Neighbors within radius search at (384.369 386.372 300.143) with radius=193.159
//     451.114 363.553 229.213 (squared distance: 10006.7)
//     452.721 398.977 207.076 (squared distance: 13492.4)
//     276.971 343.368 327.111 (squared distance: 14111)
//     447.772 491.576 334 (squared distance: 16234.2)
//     410.276 386.502 175.288 (squared distance: 16259.9)
//     445.201 488.637 253.626 (squared distance: 16322.6)
//     257.128 435.577 321.633 (squared distance: 19073.3)
//     312.382 389.858 178.369 (squared distance: 20023)
//     439.166 412.965 167.177 (squared distance: 21390)
//     482.03 493.711 266.617 (squared distance: 22183.4)
//     521.246 429.693 256.285 (squared distance: 22535.5)
//     512.669 307.615 320.109 (squared distance: 23062.3)
//     529.352 399.11 354.953 (squared distance: 24186.5)
//     292.235 430.095 178.472 (squared distance: 25204.2)
//     468.447 428.21 169.938 (squared distance: 25772.9)
//     486.896 429.648 184.04 (squared distance: 25864.7)
//     484.24 305.494 397.702 (squared distance: 26033.2)
//     285.843 501.384 233.359 (squared distance: 27395.3)
//     314.749 252.534 227.244 (squared distance: 28073.7)
//     370.492 316.152 146.118 (squared distance: 28846.8)
//     272.179 257.047 296.281 (squared distance: 29326.1)
//     424.155 426.806 135.94 (squared distance: 30180.5)
//     535.554 370.488 215.772 (squared distance: 30227.8)
//     237.352 309.898 353.828 (squared distance: 30344.3)
//     222.959 424.764 356.473 (squared distance: 30700.3)
//     505.552 501.396 241.199 (squared distance: 31390.5)
//     519.049 502.714 303.867 (squared distance: 31688.1)
//     374.487 562.587 337.03 (squared distance: 32510.2)
//     471.023 434.838 140.425 (squared distance: 35367.6)
//     204.779 353.992 361.033 (squared distance: 37008.4)
//     219.731 482.102 272.668 (squared distance: 37024.6)


