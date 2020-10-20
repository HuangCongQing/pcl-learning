/*
 * @Description: 代码片段（此文件不可运行）
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 10:18:35
 * @LastEditTime: 2020-10-20 10:22:14
 * @FilePath: /pcl-learning/CodeSnippets代码片段/smippets.cpp
 */


// 获取参数
// 附带参数读取点云文件
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(argv[1], *cloud);