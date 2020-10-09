cmake_minimum_required ( VERSION 2.6 FATAL_ERROR)   #对于cmake版本的最低版本的要求
project(1write_pcd)                                        #建立的工程名，例如源代码目录路径的变量名为CH_DIR
                                                    #工程存储目录变量名为CH_BINARY_DIR
#要求工程依赖的PCL最低版本为1.3，并且版本至少包含common和IO两个模块  这里的REQUIRED意味着如果对应的库找不到 则CMake配置的过程将完全失败，
#因为PCL是模块化的，也可以如下操作：
#           一个组件  find_package(PCL 1.6 REQUIRED COMPONENTS  io)
#           多个组件  find_package(PCL 1.6 REQUIRED COMPONENTS commom io)
#           所有组件  find_package(PCL 1.6 REQUIRED )                    
find_package(PCL 1.3 REQUIRED)  


#下面的语句是利用CMake的宏完成对PCL的头文件路径和链接路径变量的配置和添加，如果缺少下面几行，生成文件的过程中就会提示
#找不到相关的头文件，在配置CMake时，当找到了安装的PCL，下面相关的包含的头文件，链接库，路径变量就会自动设置
#                    PCL_FOUND:如果找到了就会被设置为1 ，否则就不设置
#                    PCL_INCLUDE_DIRS:被设置为PCL安装的头文件和依赖头文件的目录
#                    PCL_LIBRARIES:被设置成所建立和安装的PCL库头文件
#                    PCL_LIBRARIES_DIRS:被设置成PCL库和第三方依赖的头文件所在的目录
#                    PCL_VERSION:所找到的PCL的版本
#                    PCL_COMPONENTS:列出所有可用的组件
#                    PCL_DEFINITIONS:列出所需要的预处理器定义和编译器标志
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

#这句话告诉CMake从单个源文件write_pcd建立一个可执行文件
add_executable(write_pcd write_pcd.cpp)
#虽然包含了PCL的头文件，因此编译器知道我们现在访问所用的方法，我们也需要让链接器知道所链接的库，PCL找到库文件由
#PCL_COMMON_LIBRARIES变量指示，通过target_link_libraries这个宏来出发链接操作
# target_link_libraries(write_pcd ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}) # 链接common和IO  会报错
target_link_libraries(write_pcd ${PCL_LIBRARIES}) # 链接PCL所有库