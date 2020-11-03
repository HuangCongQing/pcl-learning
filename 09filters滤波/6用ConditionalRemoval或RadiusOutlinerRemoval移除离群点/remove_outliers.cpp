/*
 * @Description: 使用ConditionalRemoval  或RadiusOutlinerRemoval移除离群点    https://www.cnblogs.com/li-yao7758258/p/6473304.html
 * https://blog.csdn.net/qq_37124765/article/details/82262863
 * 运行：（别忘了携带参数-r   或者  -c）
 * 删除点云中不符合用户指定的一个或多个条件的数据点。
 *  ./remove_outliers -r
 * ./remove_outliers -c
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 15:50:44
 * @LastEditTime: 2020-11-03 11:41:25
 * @FilePath: /pcl-learning/09filters滤波/6用ConditionalRemoval或RadiusOutlinerRemoval移除离群点/remove_outliers.cpp
 */
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc, char **argv)
{
  if (argc != 2) //确保输入的参数
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  //填充点云
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  if (strcmp(argv[1], "-r") == 0)
  {                                                  // RadiusOutlierRemoval
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem; //创建滤波器

    outrem.setInputCloud(cloud);       //设置输入点云
    outrem.setRadiusSearch(0.8);       //设置半径为0.8的范围内找临近点
    outrem.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除
    // apply filter
    outrem.filter(*cloud_filtered); //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
  }
  else if (strcmp(argv[1], "-c") == 0)
  { //RadiusOutlierRemoval
    //创建条件限定的下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>()); //创建条件定义对象
    //添加在Z字段上大于0的比较算子
    //GT greater than
    //EQ equal
    //LT less than
    //GE greater than or equal
    //LE less than

    //为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0))); //添加在Z字段上大于0的比较算子

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8))); //添加在Z字段上小于0.8的比较算子
    // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);   //输入点云
    condrem.setKeepOrganized(true); //设置保持点云的结构
                                    // 设置是否保留滤波后删除的点，以保持点云的有序性，通过setuserFilterValue设置的值填充点云；或从点云中删除滤波后的点，从而改变其组织结构
                                    // 如果设置为true且不设置setUserFilterValue的值，则用nan填充点云
                                    //  https://blog.csdn.net/qq_37124765/article/details/82262863

    // 执行滤波
    condrem.filter(*cloud_filtered); //大于0.0小于0.8这两个条件用于建立滤波器
  }
  else
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
              << cloud->points[i].y << " "
              << cloud->points[i].z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
              << cloud_filtered->points[i].y << " "
              << cloud_filtered->points[i].z << std::endl;
  return (0);
}