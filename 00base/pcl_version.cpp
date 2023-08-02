/*
 * @Description: 
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2023-08-02 21:52:54
 * @LastEditTime: 2023-08-02 21:53:35
 * @FilePath: /pcl-learning/00base/pcl_version.cpp
 */
#include <iostream>
#include <pcl/pcl_config.h>

int main()
{
    std::cout << "PCL version: " << PCL_VERSION_PRETTY << std::endl;
    
    return 0;
}
