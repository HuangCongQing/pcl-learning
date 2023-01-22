#include "KMeans.h"
#include <random>
#include <numeric> // std::iota 
#include <algorithm>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

void KMeans::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::Indices>& cluster_idx)
{
	// -------------------------最远点采样选取聚类中心点----------------------------
	std::vector<int> selected_indices;
	selected_indices.reserve(m_clusterNum);
	const size_t num_points = cloud->size();
	std::vector<float> distances(num_points, std::numeric_limits<float>::infinity());
	size_t farthest_index = 0;
	for (size_t i = 0; i < m_clusterNum; i++) 
	{
		selected_indices.push_back(farthest_index);
		const pcl::PointXYZ& selected = cloud->points[farthest_index];
		double max_dist = 0;
		for (size_t j = 0; j < num_points; j++)
		{
			float dist = (cloud->points[j].getVector3fMap() - selected.getVector3fMap()).squaredNorm();
			distances[j] = std::min(distances[j], dist);
			if (distances[j] > max_dist) 
			{
				max_dist = distances[j];
				farthest_index = j;
			}
		}
	}
	// 获取聚类中心点
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_center(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, selected_indices, *m_center);
	// -----------------------------------进行KMeans聚类--------------------------------
	if (!cloud->empty() && !m_center->empty())
	{
		int iterations = 0;
		double sum_diff = 0.2;
		//如果大于迭代次数或者两次重心之差小于0.02就停止
		while (!(iterations >= m_maxIteration || sum_diff <= 0.02))
		{
			sum_diff = 0;
			std::vector<int> points_processed(cloud->points.size(), 0);
			cluster_idx.clear();
			cluster_idx.resize(m_clusterNum);
			for (size_t i = 0; i < cloud->points.size(); ++i)
			{
				if (!points_processed[i])
				{
					std::vector<double>dists(0, 0);
					for (size_t j = 0; j < m_clusterNum; ++j)
					{
						// 计算所有点到聚类中心点的欧式聚类
						dists.emplace_back(pcl::euclideanDistance(cloud->points[i], m_center->points[j]));
					}
					std::vector<double>::const_iterator min_dist = std::min_element(dists.cbegin(), dists.cend());
					int it = std::distance(dists.cbegin(), min_dist); // 获取最小值所在的位置
					cluster_idx[it].push_back(i);                     // 放进最小距离所在的簇
					points_processed[i] = 1;
				}
				else
				{
					continue;
				}		
			}
			// 重新计算簇中心点
			pcl::PointCloud<pcl::PointXYZ> new_centre;
			for (size_t k = 0; k < m_clusterNum; ++k)
			{
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*cloud, cluster_idx.at(k), centroid);
				pcl::PointXYZ center{ centroid[0] ,centroid[1] ,centroid[2] };
				new_centre.points.push_back(center);
			}

			//计算聚类中心点的变化量
			for (size_t s = 0; s < m_clusterNum; ++s)
			{
				sum_diff += pcl::euclideanDistance(new_centre.points[s], m_center->points[s]);
			}
			m_center->points.clear();
			*m_center = new_centre;
			++iterations;
		}
	}
}

