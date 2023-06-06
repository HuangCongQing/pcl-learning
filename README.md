<!--
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-04 18:17:00
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-12-05 23:41:31
-->
# pcl
![](https://img.shields.io/badge/pcl-learning-v0.1-brightgreen)
![](https://img.shields.io/badge/pcl->=v1.9-red)

[![GitHub stars](https://img.shields.io/github/stars/HuangCongQing/pcl-learning.svg?style=social&label=Stars)](https://github.com/murufeng/awesome_lightweight_networks)
[![GitHub forks](https://img.shields.io/github/forks/HuangCongQing/pcl-learning.svg?style=social&label=Forks)](https://github.com/murufeng/awesome_lightweight_networks)
![visitors](https://visitor-badge.glitch.me/badge?page_id=HuangCongQing/pcl-learning) 

PCL（Point Cloud Library）点云库  **个人开发环境：Ubuntu18.04**

**墙裂建议先看下：[PCL(Point Cloud Library)学习指南&资料推荐](https://zhuanlan.zhihu.com/p/268524083)**

**<font color='red'>PCL学习入门指南&代码实践(最新版)入门视频： </font> https://www.bilibili.com/video/BV1HS4y1y7AB**

**代码对应系列笔记：[PCL(Point Cloud Library)学习记录（2023）](https://www.yuque.com/huangzhongqing/pcl)**

**PCL微信交流群二维码每周都更新一次，请关注公众号【双愚】后台回复PCL加群**


![image](https://user-images.githubusercontent.com/20675770/174856411-ca460d1a-d748-4b51-b8b5-b6259394ec0a.png)

本人创建星球 **【自动驾驶感知(PCL/ROS+DL)】** 专注于自动驾驶感知领域，包括传统方法(PCL点云库,ROS)和深度学习（目标检测+语义分割）方法。同时涉及Apollo，Autoware(基于ros2)，BEV感知，三维重建，SLAM(视觉+激光雷达) ，模型压缩（蒸馏+剪枝+量化等），自动驾驶模拟仿真，自动驾驶数据集标注&数据闭环等自动驾驶全栈技术，欢迎扫码二维码加入，一起登顶自动驾驶的高峰！
![image](https://github.com/HuangCongQing/HuangCongQing/assets/20675770/304e0c4d-89d2-4cee-a2a9-3c690611c9d9)


**相关项目实战:**

* 3D-MOT(多目标检测和追踪):
  [https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking/tree/kitti](https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking/tree/kitti)
    * 需要学习ROS：https://github.com/HuangCongQing/ROS

@[双愚](https://github.com/HuangCongQing/pcl-learning) , 若fork或star请注明来源

> * 点云数据的处理可以采用获得广泛应用的Point Cloud Library (点云库，PCL库)。
> * PCL库是一个最初发布于2013年的开源C++库。它实现了大量点云相关的通用算法和高效的数据管理。
> * 支持多种操作系统平台，可在Windows、Linux、Android、Mac OS X、部分嵌入式实时系统上运行。如果说OpenCV是2D信息获取与处理的技术结晶，那么PCL在3D信息获取与处理上，就与OpenCV具有同等地位
> * PCL是BSD授权方式，可以免费进行商业和学术应用。

* 英文官网：https://pcl.readthedocs.io/projects/tutorials/en/latest/#
  * https://pointclouds.org/
* GitHub：https://github.com/PointCloudLibrary/pcl
  * 学习基于pcl1.9.1：https://github.com/PointCloudLibrary/pcl/tree/pcl-1.9.1

**Tips:**

* ubuntu下使用PCL，需要写**CMakeLists.txt**文件，然后编译才可以生成可执行文件.
* 可执行文件在build文件夹下，所以运行可执行文件时，后面添加参数的pcd文件，应放在build文件夹下才能获取到。**（注意文件路径）**
* `make -j `   (-j 自动多线程， -j4 四线程)

## 目录contents

> ***建议必学**

* [00base](00base)

##### step1

* [01common](01common )

##### step2

* [* 02kdtree k维tree](02kdtree)
* [* 03octree 八叉树](03octree)
* [* 04search(TODO)](04search):
* [05sample consensus  抽样一致性模块](05sampleconsensus抽样一致性模块)
* [06range-images深度图像](06range-images深度图像)

##### step3（must）

* [* 08 io 输入输出](08IO输入输出)
* [* 09 filters 滤波](09filters滤波)
* [* 10 features 特征](10features特征)

##### step4（根据个人需要）

* [11 surface表面 ](11surface表面 )
* [12 segmentation分割](12segmentation分割)
* [13 recognition识别](13recognition识别)
* [14 registration配准](14registration配准)
* [15 visualization可视化](15visualization可视化)
* [16 keypoints关键点](16keypoints关键点)
* [07tracking跟踪](07tracking跟踪/tracking.md)

## 编译过程

```shell
mkdir build
cd build
cmake .. // 对上一级进行编译
make  // 生成可执行文件命令
./executedemo  // 运行可执行文件
```

## 实战项目

不理解的地方,欢迎提issue: https://github.com/HuangCongQing/pcl-learning/issues

* 3D-MOT(多目标检测和追踪)
  * https://github.com/HuangCongQing/3D-LIDAR-Multi-Object-Tracking/tree/kitti
* 3D点云目标检测&语义分割-SOTA方法,代码,论文,数据集等
  * https://github.com/HuangCongQing/3D-Point-Clouds

## 相关链接

* 公众号：点云PCL
* https://github.com/Yochengliu/awesome-point-cloud-analysis
* https://github.com/QingyongHu/SoTA-Point-Cloud
* https://github.com/PointCloudLibrary/pcl
* 参考书籍：点云库PCL学习教程，朱德海，北京航空航天大学出版社
* Plus：ROS学习-https://github.com/HuangCongQing/ROS

**入门资料：**
- **<font color='red'>PCL学习入门指南&代码实践(最新版)入门视频： </font> https://www.bilibili.com/video/BV1HS4y1y7AB**
- **视频**：[bilibili-PCL点云库官网教程](https://space.bilibili.com/504859351/channel/detail?cid=130387)
- **点云库PCL学习教程书籍每章总结：**[https://github.com/MNewBie/PCL-Notes](https://github.com/MNewBie/PCL-Notes)
- 百度网盘资料：

链接：[https://pan.baidu.com/s/1ziq8s_kj5QpM8eXO_d6RJg](https://pan.baidu.com/s/1ziq8s_kj5QpM8eXO_d6RJg)<br />提取码：g6ny<br />

**代码实践资料：**

- 官方各模块示例(和对应的对象函数对照着看)【英文】：[https://pcl.readthedocs.io/projects/tutorials/en/latest/#](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)
- 官方各模块对应的对象和函数【英文】：
  - [https://pointclouds.org/documentation/modules.html](https://pointclouds.org/documentation/modules.html)
  - [https://pointclouds.org/](https://pointclouds.org/) 点击网站中的12宫图，没一格对应一个模块的对象函数详解
- [黑马机器人系列文档：PCL-3D点云](http://robot.czxy.com/docs/pcl/)：[http://robot.czxy.com/docs/pcl/](http://robot.czxy.com/docs/pcl/)
- [CSDN博主系列文章PCL学习(64篇)](https://www.cnblogs.com/li-yao7758258/category/954066.html)：[https://www.cnblogs.com/li-yao7758258/category/954066.html](https://www.cnblogs.com/li-yao7758258/category/954066.html)

## Citation 
If you find this project useful in your research, please consider cite:


```
@misc{pcl-learning2020,
    title={A Complete Study Guide on How to Learn PCL (Point Cloud Library).},
    author={Chongqing, Huang},
    howpublished = {\url{https://github.com/HuangCongQing/pcl-learning}},
    year={2020}
}
```



微信公众号：**【双愚】**（huang_chongqing） 聊科研技术,谈人生思考,欢迎关注~

![image](https://user-images.githubusercontent.com/20675770/169835565-08fc9a49-573e-478a-84fc-d9b7c5fa27ff.png)

**往期推荐：**
1. [本文不提供职业建议，却能助你一生](https://mp.weixin.qq.com/s/rBR62qoAEeT56gGYTA0law)
2. [聊聊我们大学生面试](https://mp.weixin.qq.com/s?__biz=MzI4OTY1MjA3Mg==&mid=2247484016&idx=1&sn=08bc46266e00572e46f3e5d9ffb7c612&chksm=ec2aae77db5d276150cde1cb1dc6a53e03eba024adfbd1b22a048a7320c2b6872fb9dfef32aa&scene=178&cur_album_id=2253272068899471368#rd)
3. [清华大学刘知远：好的研究方法从哪来](https://mp.weixin.qq.com/s?__biz=MzI4OTY1MjA3Mg==&mid=2247486340&idx=1&sn=6c5f69bb37d91a343b1a1e7f6929ddae&chksm=ec2aa783db5d2e95ba4c472471267721cafafbe10c298a6d5fae9fed295f455a72f783872249&scene=178&cur_album_id=1855544495514140673#rd)

### License

Copyright (c) [双愚](https://github.com/HuangCongQing/pcl-learning). All rights reserved.

Licensed under the [MIT](./LICENSE) License.


PLus: 本人创建星球 **【自动驾驶感知(PCL/ROS+DL)】** 专注于自动驾驶感知领域，包括传统方法(PCL点云库,ROS)和深度学习（目标检测+语义分割）方法。同时涉及Apollo，Autoware(基于ros2)，BEV感知，三维重建，SLAM(视觉+激光雷达) ，模型压缩（蒸馏+剪枝+量化等），自动驾驶模拟仿真，自动驾驶数据集标注&数据闭环等自动驾驶全栈技术，欢迎扫码二维码加入，一起登顶自动驾驶的高峰！
![image](https://github.com/HuangCongQing/HuangCongQing/assets/20675770/304e0c4d-89d2-4cee-a2a9-3c690611c9d9)


**最后，如果您想要支持我的工作，请扫描下面的二维码**

![image](https://user-images.githubusercontent.com/20675770/174442478-705129f7-ca4d-4e89-9b21-7e1b84817940.png)


