/*
 * @Description: 基于多项式重构的平滑和法线估计¶
http://robot.czxy.com/docs/pcl/chapter04/resampling/#_2   推荐
https://www.cnblogs.com/li-yao7758258/p/6497446.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-21 18:08:55
 * @LastEditTime: 2020-10-21 20:15:10
 * @FilePath: /pcl-learning/11surface表面 /1基于多项式重构的平滑和法线估计/resampling.cpp
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>

int
main(int argc, char **argv) {
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    // pcl::io::loadPCDFile(argv[1], *cloud);  // 获取pcd文件
    pcl::io::loadPCDFile("../ism_train_cat.pcd", *cloud);  // 获取pcd文件

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(mls_points);  // void 	process (PointCloudOut &output) override
    // Save output
    if(mls_points.size() > 0){
        pcl::io::savePCDFileASCII("../target-mls.pcd", mls_points); //保存出错
    }else{
            std::cout << "保存数据为空." << std::endl;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");;
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

}