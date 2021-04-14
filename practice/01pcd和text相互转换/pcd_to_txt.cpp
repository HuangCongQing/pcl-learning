/*
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-04-14 19:35:58
 * @LastEditTime: 2021-04-14 19:37:34
 * @FilePath: /pcl-learning/practice/01pcd和text相互转换/pcd_to_txt.cpp
 */


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void BaseMeasure::pcd2txt(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, char *filename)
{
	FILE* wc = fopen(filename, "w");
 
	int sizepcd = cloud->points.size();
	for (int i = 0; i < sizepcd; i++)
	{
		//fprintf(stdout, "%f\t%f\t%f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);  \t 相当于空格
		fprintf(wc, "%f\t%f\t%f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);  //标准输出流   stdio.h   写
	}
 
	fclose(wc);
}