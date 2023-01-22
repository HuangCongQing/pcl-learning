#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace std;

std::vector<int> powerLineSegment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, double max_dist = 0.15, double threshould = 0.81)
{
 // 建立kd-tree索引
 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
 kdtree.setInputCloud(input_cloud);
 pcl::Indices r_indices;
 std::vector<float> neighbor_square_distance;

 std::vector<int>line_idx;

 for (size_t i = 0; i < input_cloud->points.size(); ++i)
 {
  // 如果半径max_dist邻域范围内有点，则进行计算
  if (kdtree.radiusSearch(input_cloud->points[i], max_dist, r_indices, neighbor_square_distance) > 0)

  {
   //---------------PCA计算每个点的特征值------------------------
   pcl::PCA<pcl::PointXYZ> pca;
   pca.setInputCloud(input_cloud);
   pca.setIndices(std::make_shared<const pcl::Indices>(r_indices));
   pca.getEigenValues(); // 获取特征值，特征值是按从大到小排列的

   float l1 = pca.getEigenValues()[0];
   float l2 = pca.getEigenValues()[1];
   float l3 = pca.getEigenValues()[2];
   //----------------计算每个点的线性特征------------------------
   float linear = (l1 - l2) / l1;//线性
   //----------------设置阈值提取电力线--------------------------
   if (linear > threshould)
   {
    line_idx.push_back(i);
   }
  }
  else
  {
   //如果半径max_dist邻域范围内没有点，则跳过该点。
   continue;
  }

 }

 return line_idx;

}

void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
 //---------------------------------显示点云-----------------------
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
 viewer->setWindowName("电力线提取");
 viewer->setBackgroundColor(0, 0, 0);
 viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud");
 viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered");
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered");
 //viewer->addCoordinateSystem(1.0);
 //viewer->initCameraParameters();
 while (!viewer->wasStopped())
 {
  viewer->spinOnce(100);
  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
 }
}

int main(int argc, char** argv)
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 if (pcl::io::loadPCDFile<pcl::PointXYZ>("dx.pcd", *cloud) == -1)
 {
  PCL_ERROR("Cloudn't read file!");
  return -1;
 }
 //-----------------------高程滤波分割高点云和低点云-------------------------
 printf("开始高程滤波\n");
 pcl::PointCloud<pcl::PointXYZ>::Ptr high(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PassThrough<pcl::PointXYZ> pass;
 pass.setInputCloud(cloud);
 pass.setFilterFieldName("z");
 pass.setFilterLimits(0.0, 20.0);
 pass.setNegative(true);
 pass.filter(*high);
 pcl::PointCloud<pcl::PointXYZ>::Ptr low(new pcl::PointCloud<pcl::PointXYZ>);
 pass.setNegative(false);
 pass.filter(*low);
 printf("高程滤波结束！！！！\n");
 //-----------------------------分割电力线---------------------------------
 printf("开始分割电力线\n");
 std::vector<int>power_line_idx;
 power_line_idx = powerLineSegment(high, 2);
 printf("电力线分割结束！！！\n");
 //-----------------------------提取电力线---------------------------------
 pcl::ExtractIndices<pcl::PointXYZ> extr;
 extr.setInputCloud(high);//设置输入点云
 extr.setIndices(std::make_shared<const std::vector<int>>(power_line_idx));//设置索引
 pcl::PointCloud<pcl::PointXYZ>::Ptr power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 extr.filter(*power_line_cloud);     //提取对应索引的点云
 pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
 extr.setNegative(true);
 extr.filter(*output);
 //----------------------------保存电力线---------------------------------
 pcl::io::savePCDFileASCII("line_cloud.pcd", *power_line_cloud);
 //-------------------------保存非电线部分的点云--------------------------
 pcl::PointCloud<pcl::PointXYZ>::Ptr out_power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 *out_power_line_cloud = *low + *output;
 pcl::io::savePCDFileASCII("out_line_cloud.pcd", *out_power_line_cloud);
 //---------------------------可视化结果----------------------------------
 visualize_cloud(power_line_cloud, out_power_line_cloud);

 return 0;
}