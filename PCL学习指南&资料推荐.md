# PCL(Point Cloud Library)学习指南&资料推荐

> **PCL开始上手其实有点容易迷惑，而且PCL文档最近（2020）改版了。里面很多文档和之前的都不一样了，我自己学习PCL时，看的是最新文档，也踩了很多坑，现在分享一下自己的学习方法和思路，希望对大家有所帮助。**
> 先放下个人学习代码 **（有详细中文注解）**：[https://github.com/HuangCongQing/pcl-learning](https://github.com/HuangCongQing/pcl-learning)
> 自己做了很多笔记，有时间可以整理出来，发出来！


## 目录


[PCL介绍](https://github.com/HuangCongQing/pcl-learning/blob/master/PCL%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97&%E8%B5%84%E6%96%99%E6%8E%A8%E8%8D%90.md#pcl-%E4%BB%8B%E7%BB%8D)


[PCL学习指南](https://github.com/HuangCongQing/pcl-learning/blob/master/PCL%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97&%E8%B5%84%E6%96%99%E6%8E%A8%E8%8D%90.md#pcl%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97)

* [1理解入门](https://github.com/HuangCongQing/pcl-learning/blob/master/PCL%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97&%E8%B5%84%E6%96%99%E6%8E%A8%E8%8D%90.md#1-%E7%90%86%E8%A7%A3%E5%85%A5%E9%97%A8)
* [2代码实践](https://github.com/HuangCongQing/pcl-learning/blob/master/PCL%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97&%E8%B5%84%E6%96%99%E6%8E%A8%E8%8D%90.md#2-%E4%BB%A3%E7%A0%81%E5%AE%9E%E8%B7%B5)

[经验和推荐资料总结](https://github.com/HuangCongQing/pcl-learning/blob/master/PCL%E5%AD%A6%E4%B9%A0%E6%8C%87%E5%8D%97&%E8%B5%84%E6%96%99%E6%8E%A8%E8%8D%90.md#%E7%BB%8F%E9%AA%8C%E5%92%8C%E6%8E%A8%E8%8D%90%E8%B5%84%E6%96%99%E6%80%BB%E7%BB%93)

<a name="h79c1"></a>
## PCL 介绍
首先肯定先介绍下PCL，虽然大家都大概知道了(✿◡‿◡)！如下：<br />官网和github连接先mark这了。<br />官网：[https://pointclouds.org/](https://pointclouds.org/)<br />GitHub：[https://github.com/PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl)<br />

- 点云数据的处理可以采用获得广泛应用的**Point Cloud Library (点云库，PCL库)。**
- PCL库是一个最初发布于2013年的开源C++库。它实现了大量点云相关的通用算法和高效的数据管理。
- 支持多种操作系统平台，可在Windows、Linux、Android、Mac OS X、部分嵌入式实时系统上运行。如果说OpenCV是2D信息获取与处理的技术结晶，那么**PCL在3D信息获取与处理上，就与OpenCV具有同等地位**
- PCL是BSD授权方式，可以免费进行商业和学术应用。


<br />下面是PCL架构图
> 如图PCL架构图所示，对于3D点云处理来说，PCL完全是一个的模块化的现代C++模板库。其基于以下第三方库：**Boost、Eigen、FLANN、VTK、CUDA、OpenNI、Qhull**，实现点云相关的**获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化等。**

![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603545114045-b6974734-8607-49fa-9c3f-d27a542fc8fa.png#align=left&display=inline&height=489&margin=%5Bobject%20Object%5D&name=image.png&originHeight=489&originWidth=859&size=46924&status=done&style=none&width=859)

我们主要学习的就是里面各种模块，**总体来说16个模块**（下图中有15个，少了recognition），如下：<br />官网各模块：[https://pcl.readthedocs.io/projects/tutorials/en/latest/#](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)

- [01common](https://github.com/HuangCongQing/pcl-learning/blob/master/01common)
- [02kdtree k维tree](https://github.com/HuangCongQing/pcl-learning/blob/master/02kdtree)<br />
- [03octree 八叉树](https://github.com/HuangCongQing/pcl-learning/blob/master/03octree)<br />
- [04search](https://github.com/HuangCongQing/pcl-learning/blob/master/04search)<br />
- [05sample consensus 抽样一致性模块](https://github.com/HuangCongQing/pcl-learning/blob/master/05sampleconsensus%E6%8A%BD%E6%A0%B7%E4%B8%80%E8%87%B4%E6%80%A7%E6%A8%A1%E5%9D%97)<br />
- [06range-images深度图像](https://github.com/HuangCongQing/pcl-learning/blob/master/06range-images%E6%B7%B1%E5%BA%A6%E5%9B%BE%E5%83%8F)
- [07tracking](https://github.com/HuangCongQing/pcl-learning/blob/master/17tracking) （此模块，没有官方示例代码）
- [08 io 输入输出](https://github.com/HuangCongQing/pcl-learning/blob/master/08IO%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA)
- [09 filters 滤波](https://github.com/HuangCongQing/pcl-learning/blob/master/09filters%E6%BB%A4%E6%B3%A2)
- [10 features 特征](https://github.com/HuangCongQing/pcl-learning/blob/master/10features%E7%89%B9%E5%BE%81)
- [11 surface表面](https://github.com/HuangCongQing/pcl-learning/blob/master/11surface%E8%A1%A8%E9%9D%A2)
- [12 segmentation分割](https://github.com/HuangCongQing/pcl-learning/blob/master/12segmentation%E5%88%86%E5%89%B2)
- [13 recognition识别](https://github.com/HuangCongQing/pcl-learning/blob/master/13recognition%E8%AF%86%E5%88%AB)（下图中没有）
- [14 registration配准](https://github.com/HuangCongQing/pcl-learning/blob/master/14registration%E9%85%8D%E5%87%86)
- [15 visualization可视化](https://github.com/HuangCongQing/pcl-learning/blob/master/15visualization%E5%8F%AF%E8%A7%86%E5%8C%96)
- [16 keypoints关键点](https://github.com/HuangCongQing/pcl-learning/blob/master/16keypoints%E5%85%B3%E9%94%AE%E7%82%B9)

![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603547180681-75a721c2-c5e2-43a5-a862-a3a7e212db0c.png#align=left&display=inline&height=264&margin=%5Bobject%20Object%5D&name=image.png&originHeight=848&originWidth=859&size=103106&status=done&style=none&width=267)<br />

每个模块都有依赖关系，依赖关系如下图（可以看出有四层），**最基本的就是最底层的commom模块。<br />**箭头对应的是依赖关系**，比如第二层的kdtree依赖于common；第四层的registration有四个箭头，分别是sample_consensus, kdtree, common, features。<br />
<br />
<br />
<br />![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601624011126-df5f9056-9dbe-4578-8f81-60b3aba76e27.png#align=left&display=inline&height=325&margin=%5Bobject%20Object%5D&originHeight=325&originWidth=1058&size=0&status=done&style=none&width=1058)<br />官方图<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1601623998745-094ccd65-245c-493f-adb8-6bcebc345ad7.png#align=left&display=inline&height=491&margin=%5Bobject%20Object%5D&name=image.png&originHeight=645&originWidth=979&size=340648&status=done&style=none&width=746)<br />[B站up主](https://space.bilibili.com/504859351/channel/detail?cid=130387)画的更详细的图<br />**
<a name="kHgZM"></a>
## PCL学习指南
**PCL最近（2020）改版了。里面很多文档和之前的都不一样了**，我自己学习PCL时时，是看的最新文档，也踩了很多坑，现在分享一下自己的学习方法和思路，希望对大家有所帮助。<br />

<a name="XlUPm"></a>
### 1 理解入门
> 入门资料：
> - 视频：[bilibili-PCL点云库官网教程](https://space.bilibili.com/504859351/channel/detail?cid=130387)
> - **点云库PCL学习教程书籍每章总结：**[https://github.com/MNewBie/PCL-Notes](https://github.com/MNewBie/PCL-Notes)


<br />大家应该都明白，**官方文档是最好的学习资料**，但目前没有中文版，我看有些博主也翻译了一部分，现在特意整理了下！<br />
<br />

**第一步，首先，大家什么都不了解的话，强烈推荐大家看这个视频入门**：

[bilibili-PCL点云库官网教程](https://space.bilibili.com/504859351/channel/detail?cid=130387)<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603546897353-a11e3cee-3184-41c6-9186-ff9d7183c4ac.png#align=left&display=inline&height=889&margin=%5Bobject%20Object%5D&name=image.png&originHeight=889&originWidth=1313&size=564191&status=done&style=none&width=1313)<br />
<br />
<br />
<br />**第二步，然后，有一本中文教程大家应该知道：**<br />[**点云库PCL学习教程，朱德海，北京航空航天大学出版社**](https://book.douban.com/subject/20283456/)<br />这本书好多人吐槽，因为就是英文的官方文档翻译过来的，还有一些错误信息。<br />不过作为入门还是挺好的，**里面有附带的代码demo**（就是官网上的），下面是书籍PDF版和示例代码，有需要的可以看下。
> 百度网盘资料：
> 链接：[https://pan.baidu.com/s/1ziq8s_kj5QpM8eXO_d6RJg](https://pan.baidu.com/s/1ziq8s_kj5QpM8eXO_d6RJg)
> 提取码：g6ny

![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603544933089-35c1919a-1352-4259-b6e9-9761657a040e.png#align=left&display=inline&height=207&margin=%5Bobject%20Object%5D&name=image.png&originHeight=207&originWidth=325&size=9015&status=done&style=none&width=325)<br />
<br />但是这本书我没有看，因为当时在**github上发现了一个人总结的书中的笔记，如下。**<br />**文档写的还挺好的，大家可以先看这个打基础，对PCL有个宏观的了解！**<br />[https://github.com/MNewBie/PCL-Notes](https://github.com/MNewBie/PCL-Notes)<br />**![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603545361871-c0a64259-af22-4c4d-924a-7501b9d55e1c.png#align=left&display=inline&height=529&margin=%5Bobject%20Object%5D&name=image.png&originHeight=823&originWidth=942&size=126676&status=done&style=none&width=606)**<br />
<br />
<br />经过上面两步，大家应该对PCL有所了解，然后我们就敲代码吧!<br />

<a name="Tp3Jv"></a>
### 2 代码实践
> 代码实践资料：
> - 官方各模块示例(和对应的对象函数对照着看)【英文】：[https://pcl.readthedocs.io/projects/tutorials/en/latest/#](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)
> - 官方各模块对应的对象和函数【英文】：
>    - [https://pointclouds.org/documentation/modules.html](https://pointclouds.org/documentation/modules.html)
>    - [https://pointclouds.org/](https://pointclouds.org/) 点击网站中的12宫图，没一格对应一个模块的对象函数详解
> - [黑马机器人系列文档：PCL-3D点云](http://robot.czxy.com/docs/pcl/)：[http://robot.czxy.com/docs/pcl/](http://robot.czxy.com/docs/pcl/)
> - [CSDN博主系列文章PCL学习(64篇)](https://www.cnblogs.com/li-yao7758258/category/954066.html)：[https://www.cnblogs.com/li-yao7758258/category/954066.html](https://www.cnblogs.com/li-yao7758258/category/954066.html)


<br />代码学习主要**看两块**：**官方示例demo + 每个示例中的对象函数介绍。**<br />**主要结合这两块来看：边看示例代码，边看代码里面对象函数的参数详情。**
<a name="GoSdL"></a>
#### **官方示例demo代码（官方英文，后面可以结合中文注解一起看，下面有写）**
**各模块的官方示例demo位置在网站侧边栏。如下图**<br />[https://pcl.readthedocs.io/projects/tutorials/en/latest/#](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)<br />
<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603547180681-75a721c2-c5e2-43a5-a862-a3a7e212db0c.png#align=left&display=inline&height=505&margin=%5Bobject%20Object%5D&name=image.png&originHeight=848&originWidth=859&size=103106&status=done&style=none&width=512)<br />
<br />
<br />

<a name="2bkPc"></a>
#### 各模块的函数详解
进入各模块有两种方式

1. 下面链接侧边栏也都是各模块函数详情，如下图：

[https://pointclouds.org/documentation/modules.html](https://pointclouds.org/documentation/modules.html)

2. 官网的Getting Started中的**12宫图**，**每一格对应一个模块的对象函数详解**，如下图

[https://pointclouds.org/](https://pointclouds.org/)<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603547276463-edb162a8-6107-4d12-aff5-35962567619c.png#align=left&display=inline&height=794&margin=%5Bobject%20Object%5D&name=image.png&originHeight=794&originWidth=834&size=73631&status=done&style=none&width=834)<br />
<br />
<br />


| **filters** | **features** | **keypoints** |
| --- | --- | --- |
| [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697654918-b6bc96ae-8e6b-448d-a0df-4e41dc0e6be9.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=204&size=0&status=done&style=none&width=204)](https://pointcloudlibrary.github.io/documentation/group__filters.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697653793-ee24a73d-fe8d-4a25-b8df-2e6c29a8cb8d.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=361&size=0&status=done&style=none&width=361)](https://pointcloudlibrary.github.io/documentation/group__features.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652766-b034198d-0faa-471d-b7b3-c268cbf10958.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=318&size=0&status=done&style=none&width=318)](https://pointcloudlibrary.github.io/documentation/group__keypoints.html) |
| **registration** | **kdtree** | **octree** |
| [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652491-12a7b8a9-6e4c-4a5f-8a6f-6b92eed62d32.png#align=left&display=inline&height=150&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=256&size=0&status=done&style=none&width=256)](https://pointcloudlibrary.github.io/documentation/group__registration.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652671-b9e4dbc9-a638-4f1d-ac9a-c54b110603b1.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=169&size=0&status=done&style=none&width=169)](https://pointcloudlibrary.github.io/documentation/group__kdtree.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697653372-351bf77d-03a1-43dc-835c-23c89a3f2dc8.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=153&size=0&status=done&style=none&width=153)](https://pointcloudlibrary.github.io/documentation/group__octree.html) |
| **segmentation** | **sample_consensus** | **surface** |
| [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652915-aef4626b-c97e-40a4-ac65-62ee79d03f03.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=181&size=0&status=done&style=none&width=181)](https://pointcloudlibrary.github.io/documentation/group__segmentation.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652455-df497653-f814-42ae-8baa-86b45f2ea52f.png#align=left&display=inline&height=150&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=232&size=0&status=done&style=none&width=232)](https://pointcloudlibrary.github.io/documentation/group__sample__consensus.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697653548-aee9265b-b688-43af-b693-89c3cc9bff70.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=208&size=0&status=done&style=none&width=208)](https://pointcloudlibrary.github.io/documentation/group__surface.html) |
| **recognition** | **io** | **visualization** |
| [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652564-97374462-9361-4c68-9e5f-e931d4ece661.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=240&size=0&status=done&style=none&width=240)](https://pointcloudlibrary.github.io/documentation/group__recognition.html) | [![](https://cdn.nlark.com/yuque/0/2020/jpeg/232596/1601697652432-6a31ccf7-ab5b-4293-a60e-95af316884ba.jpeg#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=245&size=0&status=done&style=none&width=245)](https://pointcloudlibrary.github.io/documentation/group__io.html) | [![](https://cdn.nlark.com/yuque/0/2020/png/232596/1601697652805-b3848eaf-dfbd-4e23-9e85-ca55b8b58f16.png#align=left&display=inline&height=100&margin=%5Bobject%20Object%5D&originHeight=150&originWidth=146&size=0&status=done&style=none&width=146)](https://pointcloudlibrary.github.io/documentation/group__visualization.html) |
| Common | [Search](https://pcl.readthedocs.io/projects/tutorials/en/latest/walkthrough.html#search) |   |
| ![pcl_logo](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/pcl_logo.png) | ![pcl_logo](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/pcl_logo.png) |   |




---



<a name="UKKPy"></a>
#### 单个模块学习举例（比如IO模块）
> **总的学习方法：**
> - **先将要学的官网IO模块的实例demo代码和IO模块函数的详解页面打开**
> - **然后将模块介绍和中文代码注释的博客打开，一起学习**


<br />首先，将官网IO模块的实例demo代码和IO模块函数的详解页面打开，**如下图：**<br />**实例demo代码（实战） 每个模块都有几个demo代码，点击进去里面有代码的解释**

- [s](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#i-o)[https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#i-o](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#i-o)

**IO模块函数class**

- [https://pointcloudlibrary.github.io/documentation/group__io.html](https://pointcloudlibrary.github.io/documentation/group__io.html)

**同时，可以结合看中文详情博客笔记：**<br />[从PCD文件写入和读取点云数据](https://www.cnblogs.com/li-yao7758258/p/6435568.html)<br />[http://robot.czxy.com/docs/pcl/chapter01/io/](http://robot.czxy.com/docs/pcl/chapter01/io/)<br />个人学习代码**（有详细中文注解）**：[https://github.com/HuangCongQing/pcl-learning](https://github.com/HuangCongQing/pcl-learning)<br />
<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603592515751-4541b975-03b9-4556-9c5e-106d0076ca8c.png#align=left&display=inline&height=244&margin=%5Bobject%20Object%5D&name=image.png&originHeight=879&originWidth=1314&size=180382&status=done&style=none&width=365)![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603592580522-10ad108e-4a86-4f0b-be3a-07d0eb2b0463.png#align=left&display=inline&height=240&margin=%5Bobject%20Object%5D&name=image.png&originHeight=875&originWidth=1384&size=137854&status=done&style=none&width=380)<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603593486229-99ecad2f-93b1-4c5f-8fb4-9bc0479210ca.png#align=left&display=inline&height=423&margin=%5Bobject%20Object%5D&name=image.png&originHeight=845&originWidth=1052&size=84489&status=done&style=none&width=526)<br />![image.png](https://cdn.nlark.com/yuque/0/2020/png/232596/1603592700701-9823691a-f4ad-43f2-9e28-4f0118729af6.png#align=left&display=inline&height=453&margin=%5Bobject%20Object%5D&name=image.png&originHeight=905&originWidth=1055&size=100756&status=done&style=none&width=527.5)<br />
<br />

> 再放下个人学习代码**（有详细中文注解）**：[https://github.com/HuangCongQing/pcl-learning](https://github.com/HuangCongQing/pcl-learning)
> 自己做了很多笔记，有时间可以整理发出来。



<a name="od4Ja"></a>
## 经验和推荐资料总结
先说下这位博主的建议，我觉得挺好的：<br />[关于如何查找和利用PCL库学习资源的一些心得](https://blog.csdn.net/shine_cherise/article/details/79285162)
> - **博主个人推荐是把PCL官网的“API Documentation”、“Tutorial”和《点云库PCL学习教程2》结合起来用效果会比较好。**如果你需要用到某项功能，先去看原版的PCL官网的“API Documentation”、“Tutorial”获取最原汁原味的“第一感觉”，然后再去看《点云库PCL学习教程2》进行“中文亲切版收割”，其中还可以顺便收割一波“理论背景与整理”。**先啃硬骨头，然后云里笑。**
> - 看例程，需要用**哪块就直接上实例熟悉代码和库**。多调试代码，慢慢就熟悉了。每个人都得经历这个过程。



<a name="KYxeJ"></a>
### 系列资料汇总
**入门资料：**

- **视频**：[bilibili-PCL点云库官网教程](https://space.bilibili.com/504859351/channel/detail?cid=130387)
- **点云库PCL学习教程书籍每章总结：**[https://github.com/MNewBie/PCL-Notes](https://github.com/MNewBie/PCL-Notes)
- 百度网盘资料：

链接：[https://pan.baidu.com/s/1ziq8s_kj5QpM8eXO_d6RJg](https://pan.baidu.com/s/1ziq8s_kj5QpM8eXO_d6RJg)<br />提取码：g6ny<br />**代码实践资料：**

- 官方各模块示例(和对应的对象函数对照着看)【英文】：[https://pcl.readthedocs.io/projects/tutorials/en/latest/#](https://pcl.readthedocs.io/projects/tutorials/en/latest/#)
- 官方各模块对应的对象和函数【英文】：
   - [https://pointclouds.org/documentation/modules.html](https://pointclouds.org/documentation/modules.html)
   - [https://pointclouds.org/](https://pointclouds.org/) 点击网站中的12宫图，没一格对应一个模块的对象函数详解
- [黑马机器人系列文档：PCL-3D点云](http://robot.czxy.com/docs/pcl/)：[http://robot.czxy.com/docs/pcl/](http://robot.czxy.com/docs/pcl/)
- [CSDN博主系列文章PCL学习(64篇)](https://www.cnblogs.com/li-yao7758258/category/954066.html)：[https://www.cnblogs.com/li-yao7758258/category/954066.html](https://www.cnblogs.com/li-yao7758258/category/954066.html)



<a name="R6FaJ"></a>
## 附每个模块学习链接

**代码自己一定要动手敲一遍**

<a name="hUbE1"></a>
### [02kdtree k维tree](https://github.com/HuangCongQing/pcl-learning/blob/master/02kdtree)
官网：[https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#kdtree](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#kdtree)

对应函数：[https://pointclouds.org/documentation/group__kdtree.html](https://pointclouds.org/documentation/group__kdtree.html)<br />


详细中文博客：[https://www.cnblogs.com/li-yao7758258/p/6437440.html](https://www.cnblogs.com/li-yao7758258/p/6437440.html)<br />[https://zhuanlan.zhihu.com/p/144991935](https://zhuanlan.zhihu.com/p/144991935)<br />

<a name="zej8b"></a>
### [03octree 八叉树](https://github.com/HuangCongQing/pcl-learning/blob/master/03octree)
[https://pcl.readthedocs.io/projects/tutorials/en/latest/#octree](https://pcl.readthedocs.io/projects/tutorials/en/latest/#octree)

- [点云压缩](https://pcl.readthedocs.io/projects/tutorials/en/latest/compression.html#octree-compression)
   - 将学习如何压缩单点云和点云流
- [使用八进制进行空间分区和搜索操作](https://pcl.readthedocs.io/projects/tutorials/en/latest/octree.html#octree-search)
   - 学习如何使用八叉树进行空间分区和最近邻居搜索
- [无组织点云数据的空间变化检测没有看](https://pcl.readthedocs.io/projects/tutorials/en/latest/octree_change.html#octree-change-detection)
   - 学习如何使用八叉树检测点云内的空间变化


<br />对应函数：[https://pointclouds.org/documentation/group__octree.html](https://pointclouds.org/documentation/group__octree.html)<br />
<br />**详细中文文档：**

- [PCL学习八叉树](https://www.cnblogs.com/li-yao7758258/p/6436117.html)
- [PCLVisualizer可视化类](https://www.cnblogs.com/li-yao7758258/p/6445127.html)


<br />

<a name="POssT"></a>
### [05sample consensus 抽样一致性模块](https://github.com/HuangCongQing/pcl-learning/blob/master/05sampleconsensus%E6%8A%BD%E6%A0%B7%E4%B8%80%E8%87%B4%E6%80%A7%E6%A8%A1%E5%9D%97)
[https://pcl.readthedocs.io/projects/tutorials/en/latest/#sample-consensus](https://pcl.readthedocs.io/projects/tutorials/en/latest/#sample-consensus)<br />[https://pointclouds.org/documentation/group__sample__consensus.html](https://pointclouds.org/documentation/group__sample__consensus.html)<br />[http://robot.czxy.com/docs/pcl/chapter02/RANSAC/](http://robot.czxy.com/docs/pcl/chapter02/RANSAC/)系列<br />[PCL采样一致性算法](https://www.cnblogs.com/li-yao7758258/p/6477007.html)<br />[PCL几种采样方法](https://www.cnblogs.com/li-yao7758258/p/6527969.html)

1. 下采样  Downsampling
1. 均匀采样：这个类基本上是相同的，但它输出的点云索引是选择的关键点在计算描述子的常见方式。
1. 增采样 ：增采样是一种表面重建方法，当你有比你想象的要少的点云数据时，增采样可以帮你恢复原有的表面（S）
1. 表面重建：深度传感器的测量是不准确的，和由此产生的点云也是存在的测量误差，比如离群点，孔等表面，可以用一个算法重建表面，遍历所有的点云和插值数据，试图重建原来的表面。比如增采样，PCL使用MLS算法和类。执行这一步是很重要的，因为由此产生的点云的法线将更准确。


<br />
<br />
<br />

<a name="uIP7i"></a>
### [06range-images深度图像](https://github.com/HuangCongQing/pcl-learning/blob/master/06range-images%E6%B7%B1%E5%BA%A6%E5%9B%BE%E5%83%8F)

<br />官方Tutorials代码：[https://pcl.readthedocs.io/projects/tutorials/en/latest/#range-images](https://pcl.readthedocs.io/projects/tutorials/en/latest/#range-images)<br />[How to create a range image from a point cloud](https://pcl.readthedocs.io/projects/tutorials/en/latest/range_image_creation.html#range-image-creation)<br />[How to extract borders from range images](https://pcl.readthedocs.io/projects/tutorials/en/latest/range_image_border_extraction.html#range-image-border-extraction)<br />
<br />[http://robot.czxy.com/docs/pcl/chapter02/range_image/](http://robot.czxy.com/docs/pcl/chapter02/range_image/)<br />[PCL深度图像(1)](https://www.cnblogs.com/li-yao7758258/p/6474699.html)<br />[PCL深度图像(2)](https://www.cnblogs.com/li-yao7758258/p/6476046.html)<br />[可视化深度图像](https://www.cnblogs.com/li-yao7758258/p/6444207.html)<br />
<br />
<br />
<br />

<a name="hlB5R"></a>
### [08 io 输入输出](https://github.com/HuangCongQing/pcl-learning/blob/master/08IO%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA)
tutorial（实战）

- [s](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#i-o)[https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#i-o](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#i-o)

documentation(class和function函数)

- [https://pointcloudlibrary.github.io/documentation/group__io.html](https://pointcloudlibrary.github.io/documentation/group__io.html)

code

- [https://github.com/HuangCongQing/pcl-learning/blob/master/08IO%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA](https://github.com/HuangCongQing/pcl-learning/blob/master/08IO%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA)


<br />详情笔记：<br />[从PCD文件写入和读取点云数据](https://www.cnblogs.com/li-yao7758258/p/6435568.html)<br />[连接两个点云中的字段或数据形成新点云以及Opennni Grabber初识](https://www.cnblogs.com/li-yao7758258/p/6435759.html)<br />
<br />

<a name="jQ6Kw"></a>
### [09 filters 滤波](https://github.com/HuangCongQing/pcl-learning/blob/master/09filters%E6%BB%A4%E6%B3%A2)
官方示例：[https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#filtering](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#filtering)<br />函数：[https://pointcloudlibrary.github.io/documentation/group__filters.html](https://pointcloudlibrary.github.io/documentation/group__filters.html)<br />
<br />中文文章：<br />[PCL滤波介绍(1)](https://www.cnblogs.com/li-yao7758258/p/6445831.html)<br />[PCL滤波介绍（2）](https://www.cnblogs.com/li-yao7758258/p/6464145.html)<br />[PCL滤波介绍(3)](https://www.cnblogs.com/li-yao7758258/p/6473304.html)<br />

<a name="4dEXM"></a>
### [10 features 特征](https://github.com/HuangCongQing/pcl-learning/blob/master/10features%E7%89%B9%E5%BE%81)
[https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#features](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#features)<br />[https://pointcloudlibrary.github.io/documentation/group__features.html](https://pointcloudlibrary.github.io/documentation/group__features.html)<br />
<br />
<br />[3D 特征点概述（1）](https://www.cnblogs.com/li-yao7758258/p/9350340.html)<br />[3D 特征点概述（2）](https://www.cnblogs.com/li-yao7758258/p/9350746.html)<br />
<br />[PCL点云特征描述与提取（1）](https://www.cnblogs.com/li-yao7758258/p/6479255.html)<br />[PCL点云特征描述与提取（2）](https://www.cnblogs.com/li-yao7758258/p/6481668.html)<br />[PCL点云特征描述与提取（3）](https://www.cnblogs.com/li-yao7758258/p/6481738.html)<br />[PCL点云特征描述与提取（4）](https://www.cnblogs.com/li-yao7758258/p/6483721.html)

<a name="FjWHT"></a>
### [11 surface表面](https://github.com/HuangCongQing/pcl-learning/blob/master/11surface%E8%A1%A8%E9%9D%A2)
[https://pcl.readthedocs.io/projects/tutorials/en/latest/#surface](https://pcl.readthedocs.io/projects/tutorials/en/latest/#surface)

- [Smoothing and normal estimation based on polynomial reconstruction](https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html#moving-least-squares)
- [Construct a concave or convex hull polygon for a plane model](https://pcl.readthedocs.io/projects/tutorials/en/latest/hull_2d.html#hull-2d)
- [Fast triangulation of unordered point clouds](https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html#greedy-triangulation)
- [Fitting trimmed B-splines to unordered point clouds](https://pcl.readthedocs.io/projects/tutorials/en/latest/bspline_fitting.html#bspline-fitting)

[https://pointcloudlibrary.github.io/documentation/group__surface.html](https://pointcloudlibrary.github.io/documentation/group__surface.html)<br />
<br />博客<br />[http://robot.czxy.com/docs/pcl/chapter04/resampling/](http://robot.czxy.com/docs/pcl/chapter04/resampling/)<br />[pcl几种表面重建_3D的博客-CSDN博客](https://blog.csdn.net/dlw__/article/details/102001232)<br />
<br />[PCL点云曲面重建（1）](https://www.cnblogs.com/li-yao7758258/p/6497446.html)<br />[PCL法线估计](https://www.cnblogs.com/li-yao7758258/p/6523419.html)

<a name="ooVfx"></a>
### [12 segmentation分割](https://github.com/HuangCongQing/pcl-learning/blob/master/12segmentation%E5%88%86%E5%89%B2)
[https://pcl.readthedocs.io/projects/tutorials/en/latest/#segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/#segmentation)<br />[https://pointclouds.org/documentation/group__segmentation.html](https://pointclouds.org/documentation/group__segmentation.html)<br />
<br />
<br />[PCL中分割_欧式分割（1）](https://www.cnblogs.com/li-yao7758258/p/6694873.html)<br />[PCL中分割方法的介绍（2）](https://www.cnblogs.com/li-yao7758258/p/6696953.html)<br />[PCL中分割方法的介绍（3）](https://www.cnblogs.com/li-yao7758258/p/6697034.html)<br />
<br />[PCL点云分割（1）](https://www.cnblogs.com/li-yao7758258/p/6496664.html)<br />[PCL点云分割（2）](https://www.cnblogs.com/li-yao7758258/p/6595387.html)<br />[PCL点云分割（3）](https://www.cnblogs.com/li-yao7758258/p/6602342.html)

<a name="auAIr"></a>
### [13 recognition识别](https://github.com/HuangCongQing/pcl-learning/blob/master/13recognition%E8%AF%86%E5%88%AB)
官网：<br />[https://pcl.readthedocs.io/projects/tutorials/en/latest/#recognition](https://pcl.readthedocs.io/projects/tutorials/en/latest/#recognition)

- [3D Object Recognition based on Correspondence Grouping](https://pcl.readthedocs.io/projects/tutorials/en/latest/correspondence_grouping.html#correspondence-grouping)
- [Implicit Shape Model](https://pcl.readthedocs.io/projects/tutorials/en/latest/implicit_shape_model.html#implicit-shape-model)
- [Tutorial: Hypothesis Verification for 3D Object Recognition](https://pcl.readthedocs.io/projects/tutorials/en/latest/global_hypothesis_verification.html#global-hypothesis-verification)

[https://pointclouds.org/documentation/group__recognition.html](https://pointclouds.org/documentation/group__recognition.html)<br />
<br />博文：<br />[https://github.com/Ewenwan/MVision/tree/master/PCL_APP/Recognition](https://github.com/Ewenwan/MVision/tree/master/PCL_APP/Recognition)<br />
<a name="9WTdM"></a>
### [14 registration配准](https://github.com/HuangCongQing/pcl-learning/blob/master/14registration%E9%85%8D%E5%87%86)
[https://pcl.readthedocs.io/projects/tutorials/en/latest/#registration](https://pcl.readthedocs.io/projects/tutorials/en/latest/#registration)<br />[https://pointcloudlibrary.github.io/documentation/group__registration.html](https://pointcloudlibrary.github.io/documentation/group__registration.html)

- [The PCL Registration API官网理论](https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api)
- [使用迭代最近点算法(ICP)](http://robot.czxy.com/docs/pcl/chapter03/registration/#icp) [How to use iterative closest point](https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html#iterative-closest-point)
- 如何逐步匹配多幅点云 [How to incrementally register pairs of clouds](https://pcl.readthedocs.io/projects/tutorials/en/latest/pairwise_incremental_registration.html#pairwise-incremental-registration)
- [04-配准之交互式ICP](http://robot.czxy.com/docs/pcl/chapter03/registration_interactive_icp/) [Interactive Iterative Closest Point](https://pcl.readthedocs.io/projects/tutorials/en/latest/interactive_icp.html#interactive-icp)
- [正态分布变换配准(NDT)](http://robot.czxy.com/docs/pcl/chapter03/registration/#ndt) [How to use Normal Distributions Transform](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_distributions_transform.html#normal-distributions-transform)
- 手持式小型物品扫描仪 [In-hand scanner for small objects](https://pcl.readthedocs.io/projects/tutorials/en/latest/in_hand_scanner.html#in-hand-scanner)
- [刚性物体的鲁棒姿态估计](http://robot.czxy.com/docs/pcl/chapter03/registration/#_9) [Robust pose estimation of rigid objects](https://pcl.readthedocs.io/projects/tutorials/en/latest/alignment_prerejective.html#alignment-prerejective)


<br />[PCL学习(64)](https://www.cnblogs.com/li-yao7758258/category/954066.html)

- [PCL点云配准（1）](https://www.cnblogs.com/li-yao7758258/p/6489585.html)代码
- [PCL点云配准（2）](https://www.cnblogs.com/li-yao7758258/p/6554582.html)代码
- [PCL点云配准（3）理论](https://www.cnblogs.com/li-yao7758258/p/6605719.html)
- [PCL特征点与配准（1）](https://www.cnblogs.com/li-yao7758258/p/6612856.html)
- [点云NDT配准方法介绍](https://www.cnblogs.com/li-yao7758258/p/10705228.html)


<br />**黑马机器人 | PCL-3D点云**

- [01-点云中的数学](http://robot.czxy.com/docs/pcl/chapter03/point_cloud_math/)
- [02-点云配准原理概述](http://robot.czxy.com/docs/pcl/chapter03/registration_intro/)
- [03-点云配准流程示例 *](http://robot.czxy.com/docs/pcl/chapter03/registration/)
   - [使用迭代最近点算法(ICP)](http://robot.czxy.com/docs/pcl/chapter03/registration/#icp)
   - [正态分布变换配准(NDT)](http://robot.czxy.com/docs/pcl/chapter03/registration/#ndt)
   - [刚性物体的鲁棒姿态估计](http://robot.czxy.com/docs/pcl/chapter03/registration/#_9)
- [04-配准之交互式ICP](http://robot.czxy.com/docs/pcl/chapter03/registration_interactive_icp/)
- [05-点云配准数学原理](http://robot.czxy.com/docs/pcl/chapter03/registration_theory/)


<br />
<br />[PCL中的点云ICP配准（附源代码和数据）_qq_29462849的 ...](https://blog.csdn.net/qq_29462849/article/details/85080518)

<a name="59nEG"></a>
### [15 visualization可视化](https://github.com/HuangCongQing/pcl-learning/blob/master/15visualization%E5%8F%AF%E8%A7%86%E5%8C%96)
示例：[https://pcl.readthedocs.io/projects/tutorials/en/latest/#visualization](https://pcl.readthedocs.io/projects/tutorials/en/latest/#visualization)<br />类/对象/函数：[https://pointcloudlibrary.github.io/documentation/group__visualization.html](https://pointcloudlibrary.github.io/documentation/group__visualization.html)<br />代码：

- [PCL 可视化](https://www.cnblogs.com/li-yao7758258/p/6442156.html)
- [PCLVisualizer可视化类](https://www.cnblogs.com/li-yao7758258/p/6445127.html)
- [可视化深度图像](https://www.cnblogs.com/li-yao7758258/p/6444207.html)



<a name="E0UfL"></a>
### [16 keypoints关键点](https://github.com/HuangCongQing/pcl-learning/blob/master/16keypoints%E5%85%B3%E9%94%AE%E7%82%B9)


[https://pcl.readthedocs.io/projects/tutorials/en/latest/#keypoints](https://pcl.readthedocs.io/projects/tutorials/en/latest/#keypoints)<br />[https://pointclouds.org/documentation/group__keypoints.html](https://pointclouds.org/documentation/group__keypoints.html)<br />


博客<br />[http://robot.czxy.com/docs/pcl/chapter02/keypoints/](http://robot.czxy.com/docs/pcl/chapter02/keypoints/)<br />[PCL关键点（1）](https://www.cnblogs.com/li-yao7758258/p/6476359.html)<br />
<br />
<br />

