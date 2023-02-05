/* 

 ./01PointXYZlidar2image  /home/hcq/data/01project/wire_dataset/20230112camera_lidar_南京现场采集/20230112/51410110E0C24609B50084B5D8ACA393.pcd
*/


#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <vector>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
	 pcl::PointCloud<pcl::PointXYZ> pointcloud;
	//读入点云
	// if (pcl::io::loadPCDFile("/home/hcq/data/01project/wire_dataset/20221221new_file.pcd", pointcloud) < 0)
	if (pcl::io::loadPCDFile(argv[1], pointcloud) < 0)
	{
		PCL_ERROR("目标文件不存在！\n");
		return -1;
	}
	
	//去除nan点
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(pointcloud, pointcloud, mapping);
	//获取点云最值
	pcl::PointXYZ min;//用于存放三个轴的最小值
	pcl::PointXYZ max;//用于存放三个轴的最大值
	pcl::getMinMax3D(pointcloud, min, max);

	cout << "min.x = " << min.x << "\n" << endl;
	cout << "max.x = " << max.x << "\n" << endl;
	cout << "min.y = " << min.y << "\n" << endl;
	cout << "max.y = " << max.y << "\n" << endl;
	cout << "min.z = " << min.z << "\n" << endl;
	cout << "max.z = " << max.z << "\n" << endl;

	//定义图像的宽高
	int image_rows = max.x - min.x+1 ;
	int image_cols = max.y - min.y +1;

    cv::Mat image1(image_rows, image_cols, CV_8UC1);

	//初始化图像像素为255
	for (int i = 0; i < image1.rows; i++)
	{
		for (int j = 0; j < image1.cols; j++)
		{
			image1.at<uchar>(i, j) = 0;	
		}
	}
	//根据点云高度对图像进行赋值
	for (int i = 0; i < pointcloud.points.size(); i++)
	{    
		int image_i, image_j;
		
		
		if (pointcloud.points[i].x < 0)
		{
			 image_i = - min.x + pointcloud.points[i].x ;
		}
		else 
		{
			 image_i = - min.x + pointcloud.points[i].x; //
		}
		if (pointcloud.points[i].y < 0)
	    {
             image_j = - min.y + pointcloud.points[i].y;
		}
		else
		{
			 image_j = - min.y + pointcloud.points[i].y;
		}
		if( (pointcloud.points[i].z > 0)|| (pointcloud.points[i].z < 0)|| (pointcloud.points[i].z == 0))
		{
			if (pointcloud.points[i].z - min.z < 255)
			{
				//根据点云高度，进行0-255转换
				image1.at<uchar>(image_i, image_j) = uchar((pointcloud.points[i].z - min.z) / (max.z-min.z)*255);
				
			}
		}
		else
		{
			image1.at<uchar>(image_i, image_j) = 0;
		}	
	}
	//对图像像素点进行打印
	std::cout << image1 << std::endl;
	// imwrite("0-255_level.bmp", image1);
	imwrite("0-255_level.png", image1);
	imshow("生成图像", image1);
	waitKey(0);
	return (0);
}
