# pcl
PCL（Point Cloud Library）点云库  个人开发环境：Ubuntu18.04


>* 点云数据的处理可以采用获得广泛应用的Point Cloud Library (点云库，PCL库)。
>*  PCL库是一个最初发布于2013年的开源C++库。它实现了大量点云相关的通用算法和高效的数据管理。
>* 支持多种操作系统平台，可在Windows、Linux、Android、Mac OS X、部分嵌入式实时系统上运行。如果说OpenCV是2D信息获取与处理的技术结晶，那么PCL在3D信息获取与处理上，就与OpenCV具有同等地位
>*  PCL是BSD授权方式，可以免费进行商业和学术应用。

英文官网：https://pcl.readthedocs.io/projects/tutorials/en/latest/#
https://pointclouds.org/
中文论坛：http://www.pclcn.org/
GitHub：https://github.com/PointCloudLibrary/pcl
学习基于pcl1.9.1：https://github.com/PointCloudLibrary/pcl/tree/pcl-1.9.1



* 个人笔记：https://www.yuque.com/huangzhongqing/muam1n


### 编译过程
```shell
mkdir build
cd build
cmake .. // 对上一级进行编译
make  // 生成可执行文件命令
./pcd_write  // 运行  生成pcd文件并打印5个点的值
```


### 文章

* [3D-BoNet：比3D点云实例分割算法快10倍！代码已开源](https://mp.weixin.qq.com/s/VA593ECOsp0UDc82i8uedQ)

### 相关链接

* https://github.com/Yochengliu/awesome-point-cloud-analysis
* https://github.com/QingyongHu/SoTA-Point-Cloud
* https://github.com/PointCloudLibrary/pcl
* 参考书籍：点云库PCL学习教程，朱德海，北京航空航天大学出版社