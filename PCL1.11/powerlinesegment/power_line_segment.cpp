
#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace std;

// 分割电力线
std::vector<int> powerLineSegment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, double max_dist = 0.15, double threshould = 0.81)
{
	// 建立kd-tree索引
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(input_cloud);
	pcl::Indices r_indices;
    // pcl::PointIndices::Ptr r_indices (new pcl::PointIndices ());
	std::vector<float> neighbor_square_distance;

	std::vector<int>line_idx;

    // high 的点数量:115901 遍历点数太多耗时太长
	for (size_t i = 0; i < input_cloud->points.size(); ++i)
	{
        if(i%1000 == 0){
            std::cout<<"num: "<<i<<std::endl;
        }
		// 如果半径max_dist邻域范围内有点，则进行计算
		if (kdtree.radiusSearch(input_cloud->points[i], max_dist, r_indices, neighbor_square_distance) > 3)

		{
            // std::cout<<"r_indices.size(): "<<r_indices.size()<<std::endl; // 输入至少3个点 

			//---------------PCA计算每个点的特征值------------------------
			pcl::PCA<pcl::PointXYZ> pca;
			pca.setInputCloud(input_cloud); // 输入是所有的高点（电线点）
            // no matching function for call to ‘make_shared<<expression error> >(pcl::IndicesPtr&)’
			pca.setIndices(std::make_shared<const pcl::Indices>(r_indices)); // 输入至少3个点  [pcl::PCA::initCompute] number of points < 3
			pca.getEigenValues(); // 获取特征值，特征值是按从大到小排列的


			float l1 = pca.getEigenValues()[0];
			float l2 = pca.getEigenValues()[1];
			float l3 = pca.getEigenValues()[2];
			//----------------计算每个点的线性特征------------------------
			float linear = (l1 - l2) / l1;//线性
			//----------------设置阈值提取电力线--------------------------
            // std::cout<<"每个点的线性特征: "<<linear<<std::endl;
			if (linear > threshould) // threshould = 0.81
			{
				line_idx.push_back(i);
			}
		}
		else
		{
			//如果半径max_dist邻域范围内没有点，则跳过该点。
			continue;
		}

	}

	return line_idx;

}

void visualize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filter_cloud) {
	//---------------------------------显示点云-----------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	viewer->setWindowName("电力线提取");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ>(filter_cloud, "cloud_filtered");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered");
	//viewer->addCoordinateSystem(1.0);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/hcq/data/01project/wire_dataset/dis/first_true.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

    std::cout<<"原始点云的点数量:" << cloud->points.size()<<std::endl;

	//-----------------------高程滤波分割高点云和低点云-------------------------
	printf("开始高程滤波\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr high(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	// pass.setFilterLimits(0.0, 20.0);
	// pass.setFilterLimits(-7.0, 10.0); // 电线范围内
	pass.setFilterLimits(-100.0, -1.0); // 非电线范围内<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
    // 
	pass.setNegative(true); //设置为true，则输出范围外的点  保留范围外功能作用，是否保存滤波的限制范围内的点云，默认为false，保存限制范围点云，true时候是相反。
	pass.filter(*high); // 高点(电线点)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	
    pcl::PointCloud<pcl::PointXYZ>::Ptr low(new pcl::PointCloud<pcl::PointXYZ>);
	pass.setNegative(false); // 保留范围内 默认为false，保存限制范围点云，true时候是相反。
	pass.filter(*low); // 低点
	printf("高程滤波结束！！！！\n");

	//-----------------------------分割电力线---------------------------------
	printf("开始分割电力线\n");
    std::cout<<"high 的点数量:" << high->points.size()<<std::endl;
    std::cout<<"low 的点数量:" << low->points.size()<<std::endl;

	std::vector<int>power_line_idx;
	power_line_idx = powerLineSegment(high, 0.5);//==================================================
	printf("电力线分割结束！！！\n");

	//-----------------------------提取电力线（满足条件的power_line_idx）---------------------------------
	pcl::ExtractIndices<pcl::PointXYZ> extr;
	extr.setInputCloud(high);//设置输入点云
    //  note:   candidate expects 4 arguments, 1 provided=============================
	extr.setIndices(std::make_shared<const std::vector<int>>(power_line_idx));//设置索引
	pcl::PointCloud<pcl::PointXYZ>::Ptr power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	extr.filter(*power_line_cloud);     //提取对应索引的点云（电力线点云）
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	extr.setNegative(true);
	extr.filter(*output); // 过滤后的非电力线output
	//----------------------------保存电力线---------------------------------
	pcl::io::savePCDFileASCII("line_cloud.pcd", *power_line_cloud);
	//-------------------------保存非电线部分的点云--------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*out_power_line_cloud = *low + *output; // 非电力线+ 过滤后的非电力线output
	pcl::io::savePCDFileASCII("out_line_cloud.pcd", *out_power_line_cloud);
	//---------------------------可视化结果----------------------------------
	visualize_cloud(power_line_cloud, out_power_line_cloud);

	return 0;
}
