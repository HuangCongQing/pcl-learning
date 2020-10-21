<!--
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-04 18:17:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-10-21 16:51:24
-->
# pcl
PCL（Point Cloud Library）点云库  **个人开发环境：Ubuntu18.04**


>* 点云数据的处理可以采用获得广泛应用的Point Cloud Library (点云库，PCL库)。
>*  PCL库是一个最初发布于2013年的开源C++库。它实现了大量点云相关的通用算法和高效的数据管理。
>* 支持多种操作系统平台，可在Windows、Linux、Android、Mac OS X、部分嵌入式实时系统上运行。如果说OpenCV是2D信息获取与处理的技术结晶，那么PCL在3D信息获取与处理上，就与OpenCV具有同等地位
>*  PCL是BSD授权方式，可以免费进行商业和学术应用。

* 英文官网：https://pcl.readthedocs.io/projects/tutorials/en/latest/#
    * https://pointclouds.org/
* GitHub：https://github.com/PointCloudLibrary/pcl
    * 学习基于pcl1.9.1：https://github.com/PointCloudLibrary/pcl/tree/pcl-1.9.1



* 个人笔记：https://www.yuque.com/huangzhongqing/muam1n

Tips:
* ubuntu下使用PCL，需要写CMakeLists.txt文件，然后编译才可以生成可执行文件.
* 可执行文件在build文件夹下，所有运行可执行文件是，后面添加参数的pcd文件，应放在build文件夹下才能获取到。（所有注意文件路径）



## 目录contents
a graph of code libraries
* [00base](00base)
##### step1
* [01common](01common )

##### step2
* [02kdtree k维tree](02kdtree)
* [03octree 八叉树](03octree)
* [04search](04search)

* [05sample consensus  抽样一致性模块](05sampleconsensus抽样一致性模块)
* [06range-images深度图像](06range-images深度图像)
* [07tracking](07tracking )

##### step3
* [08 io 输入输出](08IO输入输出)
* [09 filters 滤波](09filters滤波)
* [10 features 特征](10features特征)


##### step4
* [11 surface表面 ](11surface表面 )
* [12 segmentation分割](12segmentation分割)
* [13 recognition识别](13recognition识别)
* [14 registration配准](14registration配准)
* [15 visualization可视化](15visualization可视化)
* [16 keypoints关键点](16keypoints关键点)


## 编译过程
```shell
mkdir build
cd build
cmake .. // 对上一级进行编译
make  // 生成可执行文件命令
./executedemo  // 运行可执行文件
```


## 相关链接
* 公众号：点云PCL
* https://github.com/Yochengliu/awesome-point-cloud-analysis
* https://github.com/QingyongHu/SoTA-Point-Cloud
* https://github.com/PointCloudLibrary/pcl
* 参考书籍：点云库PCL学习教程，朱德海，北京航空航天大学出版社