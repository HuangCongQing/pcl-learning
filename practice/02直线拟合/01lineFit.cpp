/*
 * @Description: 拟合直线 https://blog.csdn.net/leonardohaig/article/details/104570965
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2021-05-19 18:11:47
 * @LastEditTime: 2021-05-21 14:36:12
 * @FilePath: /c-plus-plus/practice/拟合直线并计算夹角/01拟合直线.cpp
 */


//====================================================================//
//Program:RANSAC直线拟合，并与最小二乘法结果进行对比
//Data:2020.2.29
//Author:liheng
//Version:V1.0
//====================================================================//
#include <iostream>
#include <opencv2/opencv.hpp>


//RANSAC 拟合2D 直线
//输入参数：points--输入点集
//        iterations--迭代次数
//        sigma--数据和模型之间可接受的差值,车道线像素宽带一般为10左右
//              （Parameter use to compute the fitting score）
//        k_min/k_max--拟合的直线斜率的取值范围.
//                     考虑到左右车道线在图像中的斜率位于一定范围内，
//                      添加此参数，同时可以避免检测垂线和水平线
//输出参数:line--拟合的直线参数,It is a vector of 4 floats
//              (vx, vy, x0, y0) where (vx, vy) is a normalized
//              vector collinear to the line and (x0, y0) is some
//              point on the line.
//返回值：无
void fitLineRansac(const std::vector<cv::Point2f>& points,
                   cv::Vec4f &line,
                   int iterations = 1000,
                   double sigma = 1.,
                   double k_min = -7.,
                   double k_max = 7.)
{
    unsigned int n = points.size();

    if(n<2)
    {
        return;
    }

    cv::RNG rng;
    double bestScore = -1.;
    for(int k=0; k<iterations; k++)
    {
        int i1=0, i2=0;
        while(i1==i2)
        {
            i1 = rng(n);
            i2 = rng(n);
        }
        const cv::Point2f& p1 = points[i1];
        const cv::Point2f& p2 = points[i2];

        cv::Point2f dp = p2-p1;//直线的方向向量
        dp *= 1./norm(dp);
        double score = 0;

        if(dp.y/dp.x<=k_max && dp.y/dp.x>=k_min )
        {
            for(int i=0; i<n; i++)
            {
                cv::Point2f v = points[i]-p1;
                double d = v.y*dp.x - v.x*dp.y;//向量a与b叉乘/向量b的摸.||b||=1./norm(dp)
                //score += exp(-0.5*d*d/(sigma*sigma));//误差定义方式的一种
                if( fabs(d)<sigma )
                    score += 1;
            }
        }
        if(score > bestScore)
        {
            line = cv::Vec4f(dp.x, dp.y, p1.x, p1.y);
            bestScore = score;
        }
    }
}

int main()
{
    cv::Mat image(720,1280,CV_8UC3,cv::Scalar(125,125,125));

    //以车道线参数为(0.7657,-0.6432,534,548)生成一系列点
    double k = -0.6432/0.7657;
    double b = 548 - k*534;

    std::vector<cv::Point2f> points;

    for (int i = 360; i < 720; i+=10)
    {
        cv::Point2f point(int((i-b)/k),i);
        points.emplace_back(point);
    }

    //加入直线的随机噪声
    cv::RNG rng((unsigned)time(NULL));
    for (int i = 360; i < 720; i+=10)
    {
        int x = int((i-b)/k);
        x = rng.uniform(x-10,x+10);
        int y = i;
        y = rng.uniform(y-30,y+30);
        cv::Point2f point(x,y);
        points.emplace_back(point);
    }

    //加入噪声
    for (int i = 0; i < 720; i+=20)
    {
        int x = rng.uniform(1,640);
        int y = rng.uniform(1,360);

        cv::Point2f point(x,y);
        points.emplace_back(point);
    }





    int n = points.size();
    for (int j = 0; j < n; ++j)
    {
        cv::circle(image,points[j],5,cv::Scalar(0,0,0),-1);
    }


    //RANSAC 拟合
    if(1)
    {
        cv::Vec4f lineParam;
        fitLineRansac(points,lineParam,1000,10);
        double k = lineParam[1] / lineParam[0];
        double b = lineParam[3] - k*lineParam[2];

        cv::Point p1,p2;
        p1.y = 720;
        p1.x = ( p1.y - b) / k;

        p2.y = 360;
        p2.x = (p2.y-b) / k;

        cv::line(image,p1,p2,cv::Scalar(0,255,0),2);
    }


    //最小二乘法拟合
    if(0)
    {
        cv::Vec4f lineParam;
        cv::fitLine(points,lineParam,cv::DIST_L2,0,0.01,0.01);
        double k = lineParam[1] / lineParam[0];
        double b = lineParam[3] - k*lineParam[2];

        cv::Point p1,p2;
        p1.y = 720;
        p1.x = ( p1.y - b) / k;

        p2.y = 360;
        p2.x = (p2.y-b) / k;

        cv::line(image,p1,p2,cv::Scalar(0,0,255),2);
    }
    cv::imshow("image",image);
    cv::waitKey(0);

    return 0;
}
