/*
 * @Description: 为输入数据集中的点的子集估计一组表面法线。https://pcl.readthedocs.io/projects/tutorials/en/latest/how_features_work.html#how-3d-features-work
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 15:04:50
 * @LastEditTime: 2020-10-19 18:34:17
 * @FilePath: /pcl-learning/10features特征/2估计点云子集的表面法线(error)/normal_estimation_subdset_points.cpp
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
// 下面报错 error: ‘shared_ptr’ is not a member of ‘pcl’

int main ()
 {
//打开点云代码
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile ("../table_scene_lms400.pcd", *cloud);

  // //创建一组要使用的索引。为简单起见，我们将使用云中前10％的点
  std::vector<int> indices (std::floor (cloud->size () / 10));
  for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;

  // //创建NormalEstimation类，并将输入数据集传递给它
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // 传递索引
  // std::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices)); // 报错
  pcl::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices)); // 报错
  ne.setIndices (indicesptr);

  // 创建一个空的kdtree表示形式，并将其传递给normal estimation 对象
  // 它的内容将根据给定的输入数据集填充到对象内部（因为未提供其他搜索表面）。
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // 在半径3cm的范围内近邻
  ne.setRadiusSearch (0.03);

  // 计算特征
  ne.compute (*cloud_normals);

  // cloud_normals->size () should have the same size as the input indicesptr->size ()
}