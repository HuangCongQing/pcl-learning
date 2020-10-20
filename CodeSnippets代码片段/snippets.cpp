/*
 * @Description: 代码片段（此文件不可运行）
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 10:18:35
 * @LastEditTime: 2020-10-20 15:57:18
 * @FilePath: /pcl-learning/CodeSnippets代码片段/snippets.cpp
 */

// 判断不同参数下不同代码
  if (strcmp(argv[1], "-r") == 0){     
    // code
  }
  else if (strcmp(argv[1], "-c") == 0){  
    // code
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }

// 获取参数
// 附带参数读取点云文件
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(argv[1], *cloud);