#include"KMeans.h"
#include<pcl/io/pcd_io.h>
#include<pcl/common/time.h>

using namespace std;

int main()
{
	// -------------------------------加载点云-----------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("../line.pcd", *cloud) == -1)
	{
		PCL_ERROR("读取源标点云失败 \n");
		return (-1);
	}
	cout << "从点云中读取 " << cloud->size() << " 个点" << endl;
	// ------------------------------K均值聚类-----------------------------
	pcl::StopWatch time;
	int clusterNum = 3; // 聚类个数
	int maxIter = 50;   // 最大迭代次数
	KMeans kmeans(clusterNum, maxIter);
	std::vector<pcl::Indices> cluster_indices;
	kmeans.extract(cloud, cluster_indices);
	cout << "聚类的个数为：" << cluster_indices.size() << endl;
	cout << "代码运行时间:" << time.getTimeSeconds() << "秒" << endl;

	// ---------------------------聚类结果分类保存--------------------------
	int begin = 1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dbscan_all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		// 获取每一个聚类点云团的点
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dbscan(new pcl::PointCloud<pcl::PointXYZRGB>);
		// 同一点云团赋上同一种颜色
		uint8_t R = rand() % (256) + 0;
		uint8_t G = rand() % (256) + 0;
		uint8_t B = rand() % (256) + 0;

		for (auto pit = it->begin(); pit != it->end(); ++pit)
		{
			pcl::PointXYZRGB point_db;
			point_db.x = cloud->points[*pit].x;
			point_db.y = cloud->points[*pit].y;
			point_db.z = cloud->points[*pit].z;
			point_db.r = R;
			point_db.g = G;
			point_db.b = B;
			cloud_dbscan->points.push_back(point_db);
		}
		// 聚类结果分类保存
		pcl::io::savePCDFileBinary("kmeans" + std::to_string(begin) + ".pcd", *cloud_dbscan);
		begin++;

	}

	return 0;
}


