/*
 * @Description: 代码片段（此文件不可运行）
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 10:18:35
 * @LastEditTime: 2020-10-22 10:33:08
 * @FilePath: /pcl-learning/CodeSnippets代码片段/snippets.cpp
 */

// 3 输入输出点云文件
  // 方式1：
    pcl::io::loadPCDFile("xxxx.pcd", *cloud);
    pcl::io::savePCDFileASCII("xxx.pcd", cloud); //保存出错
  // 方式2：
    pcl::PCDReader reader;
    reader.read ("../xxx.pcd", *cloud);

    pcl::PCDWriter writer;
    writer.write ("../xxx.pcd", *cloud_hull, false);



// 2 判断不同参数下不同代码
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

// 1 获取参数
// 附带参数读取点云文件
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(argv[1], *cloud);


