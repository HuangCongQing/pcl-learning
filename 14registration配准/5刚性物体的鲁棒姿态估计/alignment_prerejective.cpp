/*
 * @Description:  5刚性物体的鲁棒姿态估计  http://robot.czxy.com/docs/pcl/chapter03/registration/#_9
 * https://blog.csdn.net/zfjBIT/article/details/94566603
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-22 19:29:32
 * @LastEditTime: 2020-10-23 10:36:17
 * @FilePath: /pcl-learning/14registration配准/5刚性物体的鲁棒姿态估计/alignment_prerejective.cpp
 */

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// 首先从定义类型开始，以免使代码混乱
// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // 然后，我们实例化必要的数据容器，检查输入参数并加载对象和场景点云。 
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }

  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||  pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0) // 判断加载点云文件失败
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }

  // 1 Downsample 下采样：使用0.005提速分辨率对目标物体和场景点云进行空间下采样
  // 为了加快处理速度，我们使用PCL的：pcl::VoxelGrid类将对象和场景点云的采样率下采样至5 mm。
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);  // 对象下采样
  grid.filter (*object);
  grid.setInputCloud (scene); // 场景下采样
  grid.filter (*scene);

  // 2 估计场景法线 Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene);

  // 3 特征估计 Estimate features
  // 对于下采样点云中的每个点，我们现在使用PCL的pcl::FPFHEstimationOMP<>类来计算用于对齐过程中用于匹配的快速点特征直方图（FPFH）描述符。
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);

  // 4 对齐配准对象创建与配置  实施配准Perform alignment  
  // SampleConsensusPrerejective 实现了有效的RANSAC姿势估计循环
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align; // //基于采样一致性的位姿估计
  align.setInputSource (object); // 对象
  align.setSourceFeatures (object_features); // 对象特征
  align.setInputTarget (scene);  // 场景
  align.setTargetFeatures (scene_features); //  场景特征
  align.setMaximumIterations (50000); // 采样一致性迭代次数 Number of RANSAC iterations
  align.setNumberOfSamples (3); // 在对象和场景之间进行采样的点对应数。 至少需要3个点才能计算姿势。Number of points to sample for generating/prerejecting a pose 
  align.setCorrespondenceRandomness (5); //  使用的临近特征点的数目 Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // 多边形边长度相似度阈值 Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // 判断是否为内点的距离阈值 Inlier threshold
  align.setInlierFraction (0.25f); //  接受位姿假设所需的内点比例 Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned); // 对齐的对象存储在点云object_aligned中。 
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ()); // Inliers: 1384/3432

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment-刚性物体的鲁棒姿态估计");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }

  return (0);
}