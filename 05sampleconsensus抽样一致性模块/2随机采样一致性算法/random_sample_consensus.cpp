/*
 * @Description: 1随机采样一致性算法           http://robot.czxy.com/docs/pcl/chapter02/RANSAC/#ransac  文章代码有错，此代码已修改
 * error：error: ‘chrono_literals’ is not a namespace-name
 * bug修复：https://xbuba.com/questions/35856969
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 15:04:50
 * @LastEditTime: 2020-10-19 18:34:17
 * @FilePath: /pcl-learning/05sampleconsensus抽样一致性模块/2随机采样一致性算法/random_sample_consensus.cpp
 */

#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::literals::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr final = nullptr) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    if (final != nullptr) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(final, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(final, color_handler, "final cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "final cloud");
    }

    viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
}
/**
 * 使用方法：
 *
 * random_sample_consensus     创建包含外部点的平面
 * random_sample_consensus -f  创建包含外部点的平面，并计算平面内部点
 *
 * random_sample_consensus -s  创建包含外部点的球体
 * random_sample_consensus -sf 创建包含外部点的球体，并计算球体内部点
 */
int
main(int argc, char **argv) {
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

    // populate our PointCloud with points
    cloud->width = 500;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->points.size(); ++i) {
        if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {  // -s   -sf
            cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            if (i % 5 == 0)     // 可能会散落在球体外
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            else if (i % 2 == 0)// 在球体正方向内
                cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                                          - (cloud->points[i].y * cloud->points[i].y));
            else // 在球体负方向内
                cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                                           - (cloud->points[i].y * cloud->points[i].y));
        } else {
            cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            if (i % 2 == 0)
                cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            else
                cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }

    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    // 圆形
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    // 平面
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    if (pcl::console::find_argument(argc, argv, "-f") >= 0) {  // -f
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // copies all inliers of the model computed to another PointCloudqq
    // 将cloud中指定索引的点拷贝到final点云中
    pcl::copyPointCloud(*cloud, inliers, *final);

    // creates the visualization object and adds either our original cloud or all of the inliers
    // depending on the command line arguments specified.
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
        viewer = simpleVis(cloud, final);
    else
        viewer = simpleVis(cloud);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}