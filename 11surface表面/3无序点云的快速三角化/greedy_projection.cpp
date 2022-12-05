/*
 * @Description: 无序点云的快速三角化  https://www.cnblogs.com/li-yao7758258/p/6497446.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-22 10:35:16
 * @LastEditTime: 2022-12-05 23:18:56
 * @FilePath: /pcl-learning/11surface表面/3无序点云的快速三角化/greedy_projection.cpp
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>      //贪婪投影三角化算法

int main (int argc, char** argv)
{
  // 将一个XYZ点类型的PCD文件打开并存储到对象中
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("../bun0.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
  tree->setInputCloud (cloud); //用cloud构建tree对象
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals); //估计法线存储到其中
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);    //连接字段
  //* cloud_with_normals = cloud + normals

  //定义搜索树对象
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);   //点云构建搜索树

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
  pcl::PolygonMesh triangles;                //存储最终三角化的网络模型
 
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);  //设置连接点之间的最大距离，（即是三角形最大边长）

  // 设置各参数值
  gp3.setMu (2.5);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
  gp3.setMaximumNearestNeighbors (100);    //设置样本点可搜索的邻域个数
  gp3.setMaximumSurfaceAngle(M_PI/4); // 设置某点法线方向偏离样本点法线的最大角度45
  gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10
  gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120
  gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致

  // Get result
  gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
  gp3.setSearchMethod (tree2);   //设置搜索方式
  gp3.reconstruct (triangles);  //重建提取三角化

  // 附加顶点信息
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();


  // Finish
  return 0;
}