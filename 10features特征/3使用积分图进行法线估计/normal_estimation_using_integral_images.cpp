/*
 * @Description: 使用积分图计算一个有序的点云的法线，注意此方法只适用有序点云 : https://www.cnblogs.com/li-yao7758258/p/6479255.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 16:08:38
 * @LastEditTime: 2020-10-19 16:11:36
 * @FilePath: /pcl-learning/10features特征/3使用积分图进行法线估计/normal_estimation_using_integral_images.cpp
 */
#include <pcl/io/io.h>
     #include <pcl/io/pcd_io.h>
     #include <pcl/features/integral_image_normal.h>
     #include <pcl/visualization/cloud_viewer.h>
     int
     main ()
     {
             //打开点云
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
             pcl::io::loadPCDFile ("../table_scene_mug_stereo_textured.pcd", *cloud);
             //创建法线估计向量
             pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
             pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
         /****************************************************************************************
         三种法线估计方法
          COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
         AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
         AVERAGE_DEPTH——CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
         ********************************************************************************************/
             ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);  //设置法线估计的方式AVERAGE_3D_GRADIENT
                
             ne.setMaxDepthChangeFactor(0.02f);   //设置深度变化系数
             ne.setNormalSmoothingSize(10.0f);   //设置法线优化时考虑的邻域的大小
             ne.setInputCloud(cloud);               //输入的点云
             ne.compute(*normals);                    //计算法线
             //可视化
             pcl::visualization::PCLVisualizer viewer("PCL Viewer");   //视口的名称
             viewer.setBackgroundColor (0.0, 0.0, 0.5);    //背景颜色的设置
             viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);  //将法线加入到点云中
             while (!viewer.wasStopped ())
             {
               viewer.spinOnce ();
             }
             return 0;
     }