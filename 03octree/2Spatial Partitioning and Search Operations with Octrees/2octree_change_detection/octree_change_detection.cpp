/*
 * @Description:：无序点云数据集的空间变化检测： https://www.cnblogs.com/li-yao7758258/p/6441595.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-13 10:53:45
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-13 11:21:16
 */
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    srand((unsigned int)time(NULL)); //用系统时间初始化随机种子

    // 八叉树的分辨率，即体素的大小
    float resolution = 32.0f;

    // 初始化空间变化检测对象
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>); //创建点云实例cloudA生成的点云数据用于建立八叉树octree对象

    // 为cloudA点云填充点数据
    cloudA->width = 128;                                   //设置点云cloudA的点数
    cloudA->height = 1;                                    //无序点
    cloudA->points.resize(cloudA->width * cloudA->height); //总数

    for (size_t i = 0; i < cloudA->points.size(); ++i) //循环填充
    {
        cloudA->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 添加点云到八叉树中，构建八叉树
    octree.setInputCloud(cloudA);     //设置输入点云
    octree.addPointsFromInputCloud(); //从输入点云构建八叉树
                                      /***********************************************************************************
    点云cloudA是参考点云用其建立的八叉树对象描述它的空间分布，octreePointCloudChangeDetector
    类继承自Octree2BufBae类，Octree2BufBae类允许同时在内存中保存和管理两个octree，另外它应用了内存池
    该机制能够重新利用已经分配了的节点对象，因此减少了在生成点云八叉树对象时昂贵的内存分配和释放操作
    通过访问 octree.switchBuffers ()重置八叉树 octree对象的缓冲区，但把之前的octree数据仍然保留在内存中
   ************************************************************************************/
    // 交换八叉树的缓冲，但是CloudA对应的八叉树结构仍然在内存中
    octree.switchBuffers();
    //cloudB点云用于建立新的八叉树结构，与前一个cloudA对应的八叉树共享octree对象，同时在内存中驻留
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>); //实例化点云对象cloudB

    // 为cloudB创建点云
    cloudB->width = 128;
    cloudB->height = 1;
    cloudB->points.resize(cloudB->width * cloudB->height);

    for (size_t i = 0; i < cloudB->points.size(); ++i)
    {
        cloudB->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 添加cloudB到八叉树中
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    /**************************************************************************************************************
  为了检索获取存在于couodB的点集R，此R并没有cloudA中的元素，可以调用getPointIndicesFromNewVoxels方法，通过探测两个八叉树之间
  体素的不同，它返回cloudB 中新加点的索引的向量，通过索引向量可以获取R点集  很明显这样就探测了cloudB相对于cloudA变化的点集，但是只能探测
  到在cloudA上增加的点集，二不能探测减少的
****************************************************************************************************************/

    std::vector<int> newPointIdxVector; //存储新添加的索引的向量

    // 获取前一cloudA对应八叉树在cloudB对应在八叉树中没有的点集
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // 打印点集
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
                  << cloudB->points[newPointIdxVector[i]].y << " "
                  << cloudB->points[newPointIdxVector[i]].z << std::endl;
}