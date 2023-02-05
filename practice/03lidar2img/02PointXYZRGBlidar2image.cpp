#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vtkPlaneSource.h>  
#include <pcl/common/common.h>
#include<opencv2/opencv.hpp>


#include <pcl/io/pcd_io.h>   //PCD读写类相关的头文件
#include <pcl/point_types.h> //PCL中支持的点类型的头文件

using namespace std;

typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGB pointcolor;


//将a值缩放到0-255之间，放回的类型是整型int
int scale_to_255(int pixel_value, int min, int max) {
	return int((((pixel_value - min) / float(max - min)) * 255));
}

void use_pcdTopicture_main(pcl::PointCloud< pcl::PointXYZRGB>::Ptr cloud, float res, float side_range_left, float side_range_right, float fwd_range_behind , float fwd_range_front, float height_range_min,float height_range_max) {
	/*初始化 
	cloud        表示点云地址
	res          表示分辨率
	side_range   left - most to right - most
	fwd_range    back - most to forward - most
	height_range bottom - most to upper - most*/
	res = 1.0;
	//获取点云的最值
	pcl::PointXYZRGB min, max;
	pcl::getMinMax3D(*cloud, min, max);
	//将点云的最值设置为框住点云的边界,自动设置边框大小,目的就是构建图片大小。
	//在cloudcompare中绿色坐标轴代表y，红色坐标轴代表x，蓝色坐标轴代表z，因此我们想要的是水平面也就是XOZ面
	//根据双目成像原理我认为点云的原点就是相机的位置。，所以我认为款选的是要固定的
	float side_range [] = { -1500,800 };
	float fwd_range[] = { -500 ,500 };
	float height_range[] = { -2,2};//y轴相关
	//记录筛选出来的点云下标
	vector<int>indices;

	//获取用矩形框款住想要的点云 利用循环每一个点云，筛选出属于规定框中的点云
	for (int i = 0; i < cloud->points.size(); i++) {
		if ((cloud->points[i].x > fwd_range[0] && cloud->points[i].x < fwd_range[1]) && (cloud->points[i].z > side_range[0] && cloud->points[i].z < side_range[1]))
		{
			indices.push_back(i);
		}
	};

	//在img图像中坐标
	vector<int>x_img;
	vector<int>y_img;
	//将点云的值映射到像素位置中,分辨率为1的话则，将点云的值原封不动的映射到图像中
	for (int i = 0; i < indices.size(); i++) {
		x_img.push_back(-cloud->points[i].z/res);
		y_img.push_back(-cloud->points[i].x / res);

	}
	//平移图像数据到点云的最左上角
	for (int i = 0; i < indices.size(); i++) {
		x_img[i]-=int(floor(side_range[0]/res));
		y_img[i] += int(ceil(fwd_range[1] / res));

	}
	//将厚度信息的值转化为255值，并且填充到对应的img中对应的坐标中
	vector<int>pixel_values;
	//先筛选符合厚度条件的点云
	for (int i = 0; i < indices.size(); i++) {
		if (cloud->points[i].y > height_range[0] && cloud->points[i].y < height_range[1])
		{
			pixel_values.push_back(cloud->points[i].y);
		}
	}
	//将点云的厚度信息经过比例转化成255对应值，赋值到img图像对应的坐标值中
	for (int i = 0; i < indices.size(); i++) {
		pixel_values[i]=scale_to_255(pixel_values[i], height_range[0], height_range[1]);
	}

	//创建图像大小，取决于我们所定义的框选矩形和分辨率
	int x_max = 1 + int((side_range[1] - side_range[0]) / res);
	int y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res);
	//利用opencv创建一个Mat类型来存储图像的信息
	cv::Mat im = cv::Mat::zeros(x_max, y_max, CV_8U);
	//im[x_img, y_img]=pixel_values;
	//创建一个循环，遍历x_img或者y_img的点，给Mat类型的图像，对应的x_img，y_img坐标赋值pixel_values值
	for (int i = 0; i < x_img.size(); i++) {
		im.at<uchar>(x_img[i], y_img[i]) = pixel_values[i];
	}
	//数据图片打印信息
	cout << "图像中x轴最小点坐标值为x_img min:"<< *min_element(x_img.begin(), x_img.end()) << endl;
	cout << "图像中x轴最大点坐标值为x_img max:" << *max_element(x_img.begin(), x_img.end()) << endl;
	cout << "图像中y轴最小点坐标值为y_img min:" << *min_element(y_img.begin(), y_img.end()) << endl;
	cout << "图像中y轴最大点坐标值为y_img max:" << *max_element(y_img.begin(), y_img.end()) << endl;
	
	

	//可视化
	cvNamedWindow("img_result");
	cv::imshow("img_result", im);

	//取反
	cv::Mat im_reverse;
	im_reverse = 255 - im;
	cvNamedWindow("img_reverse_result");
	cv::imshow("img_reverse_result", im_reverse);
	cvvWaitKey();

	//将图片保存
	std::vector<int>param;
	param.push_back(CV_IMWRITE_PXM_BINARY);
	cv::imwrite("img_reverse_result.bmp", im_reverse, param);
	cout << "save img" << endl;
	
}




int main(int argc,char **argv)
{
    
    pcl::PointCloud<pointcolor>::Ptr cloud (new pcl::PointCloud<pointcolor>);
    pcl::io::loadPCDFile(argv[1],*cloud);
    // pcl::io::loadPCDFile("/home/hcq/data/01project/wire_dataset/20221221new_file.pcd",*cloud);




    // 
    /*初始化 
	cloud        表示点云地址
	res          表示分辨率
	side_range   left - most to right - most
	fwd_range    back - most to forward - most
	height_range bottom - most to upper - most
    */
    float res = 0.05;
    float side_range_left = -100;
    float side_range_right = 100;

    float fwd_range_behind = -100;
    float fwd_range_front = 100;

    float height_range_min = -100;
    float height_range_max = 100;
    use_pcdTopicture_main(cloud, res, side_range_left, side_range_right, fwd_range_behind, fwd_range_front, height_range_min, height_range_max);

    return 0;
}
