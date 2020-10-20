/*
 * @Description: 1随机采样一致性算法           https://www.cnblogs.com/li-yao7758258/p/6477007.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 15:04:50
 * @LastEditTime: 2020-10-19 18:34:17
 * @FilePath: /pcl-learning/05sampleconsensus抽样一致性模块/1随机采样一致性算法/random_sample_consensus.cpp
 */

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}
/******************************************************************************************************************
 对点云进行初始化，并对其中一个点云填充点云数据作为处理前的的原始点云，其中大部分点云数据是基于设定的圆球和平面模型计算
  而得到的坐标值作为局内点，有1/5的点云数据是被随机放置的局外点。
 *****************************************************************************************************************/
int
main(int argc, char** argv)
{
  // 初始化点云对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //存储源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);   //存储提取的局内点

  // 填充点云数据
  cloud->width    = 500;                 //填充点云数目
   cloud->height   = 1;                     //无序点云
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
//根据命令行参数用x^2+y^2+Z^2=1设置一部分点云数据，此时点云组成1/4个球体作为内点
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);   //此处对应的点为局外点
      else if(i % 2 == 0)
        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
      else
        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                        - (cloud->points[i].y * cloud->points[i].y));
    }
    else
    { //用x+y+z=1设置一部分点云数据，此时地拿云组成的菱形平面作为内点
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);   //对应的局外点
      else
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }

  std::vector<int> inliers;  //存储局内点集合的点的索引的向量

  //创建随机采样一致性对象
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));    //针对球模型的对象
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));   //针对平面模型的对象
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)
  {  //根据命令行参数，来随机估算对应平面模型，并存储估计的局内点
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);    //与平面距离小于0.01 的点称为局内点考虑，注意局内点设置参数
    ransac.computeModel();                   //执行随机参数估计
    ransac.getInliers(inliers);                 //存储估计所得的局内点
  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  { 
   //根据命令行参数  来随机估算对应的圆球模型，存储估计的内点
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // 复制估算模型的所有的局内点到final中
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

  // 创建可视化对象并加入原始点云或者所有的局内点

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }