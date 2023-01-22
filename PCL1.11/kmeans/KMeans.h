#pragma once

#include <iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include <pcl/point_types.h>

class KMeans
{
private:

	int m_maxIteration; // 最大迭代次数
	int m_clusterNum;   // 聚类个数
	//pcl::PointCloud<pcl::PointXYZ>::Ptr m_center; // 聚类中心点

public:

	KMeans(int k, int max_iteration) :
		m_clusterNum(k), m_maxIteration(max_iteration)/**, m_center(new pcl::PointCloud<pcl::PointXYZ>)*/ {}
	~KMeans() {}

	void extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::Indices>& cluster_idx);

};


