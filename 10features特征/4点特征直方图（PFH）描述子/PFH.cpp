/*
 * @Description: 点特征直方图（PFH）描述子  :https://www.cnblogs.com/li-yao7758258/p/6481668.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 16:37:14
 * @LastEditTime: 2020-10-19 18:05:09
 * @FilePath: /pcl-learning/10features特征/4点特征直方图（PFH）描述子/PFH.cpp
 */
#include <pcl/io/io.h>
#include <pcl/point_types.h>                  //点类型头文件
#include <pcl/features/pfh.h>                 //pfh特征估计类头文件
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>  // 不要忘记
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
int main(int argc, char **argv)
{
    //其他相关操作

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("../ism_test_cat.pcd", *cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

    //打开点云文件估计法线等


    //创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它
    pcl::PFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::PFHSignature125> pfh;

    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);

    //如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

    //创建一个空的kd树表示法，并把它传递给PFH估计对象。
    //基于已给的输入数据集，建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod(tree); ; //输出数据集pcl::PointCloud<pcl::PFHSignature125>::Ptrpfhs(new pcl::PointCloud<pcl::PFHSignature125>());


    //输出数据集

    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

    //使用半径在5厘米范围内的所有邻元素。


    //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
    pfh.setRadiusSearch(0.05);//计算pfh特征值


    //计算pfh特征值

    pfh.compute(*pfhs);

    // pfhs->points.size ()应该与input cloud->points.size ()有相同的大小，即每个点都有一个pfh特征向量

       // ========直方图可视化=============================
      pcl::visualization::PCLPlotter plotter;
      plotter.addFeatureHistogram(*pfhs, 300); //设置的很坐标长度，该值越大，则显示的越细致
      plotter.plot();
    return 0;
}