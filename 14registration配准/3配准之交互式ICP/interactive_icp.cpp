/*
 * @Description: 交互式ICP   http://robot.czxy.com/docs/pcl/chapter03/registration_interactive_icp/
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-22 18:45:48
 * @LastEditTime: 2020-10-22 18:46:52
 * @FilePath: /pcl-learning/14registration配准/3配准之交互式ICP/interactive_icp.cpp
 */


//
// Created by ty on 20-5-29.
//

#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
/**
 * 此函数是查看器的回调。 当查看器窗口位于顶部时，只要按任意键，就会调用此函数。 如果碰到“空格”； 将布尔值设置为true。
 * @param event
 * @param nothing
 */
void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

int
main (int argc,
      char* argv[])
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

//    我们检查程序的参数，设置初始ICP迭代的次数，然后尝试加载PLY文件。
    // Checking program arguments
    if (argc < 2)
    {
        printf ("Usage :\n");
        printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
        PCL_ERROR ("Provide one ply file.\n");
        return (-1);
    }

    int iterations = 1;  // Default number of ICP iterations
    if (argc > 2)
    {
        // If the user passed the number of iteration as an argument
        iterations = atoi (argv[2]);
        if (iterations < 1)
        {
            PCL_ERROR ("Number of initial iterations must be >= 1\n");
            return (-1);
        }
    }

    pcl::console::TicToc time;
    time.tic ();
    if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

    // 我们使用刚性矩阵变换来变换原始点云。
    // cloud_in包含原始点云。
    // cloud_tr和cloud_icp包含平移/旋转的点云。
    // cloud_tr是我们将用于显示的备份（绿点云）。

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 8;  // The angle of rotation in radians
    transformation_matrix (0, 0) = std::cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = std::cos (theta);

    // A translation on Z axis (0.4 meters)
    transformation_matrix (2, 3) = 0.4;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

    // 这是ICP对象的创建。 我们设置ICP算法的参数。
    // setMaximumIterations（iterations）设置要执行的初始迭代次数（默认值为1）。
    // 然后，我们将点云转换为cloud_icp。 第一次对齐后，我们将在下一次使用该ICP对象时（当用户按下“空格”时）将ICP最大迭代次数设置为1。

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    // 检查ICP算法是否收敛； 否则退出程序。 如果返回true，我们将转换矩阵存储在4x4矩阵中，然后打印刚性矩阵转换。
    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            // 如果用户按下键盘上的任意键，则会调用keyboardEventOccurred函数。 此功能检查键是否为“空格”。
            // 如果是，则全局布尔值next_iteration设置为true，从而允许查看器循环输入代码的下一部分：调用ICP对象以进行对齐。
            // 记住，我们已经配置了该对象输入/输出云，并且之前通过setMaximumIterations将最大迭代次数设置为1。

            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            // 和以前一样，我们检查ICP是否收敛，如果不收敛，则退出程序。
            if (icp.hasConverged ())
            {
                // printf（“ 033 [11A”）; 在终端增加11行以覆盖显示的最后一个矩阵是一个小技巧。
                // 简而言之，它允许替换文本而不是编写新行； 使输出更具可读性。 我们增加迭代次数以更新可视化器中的文本值。
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());

                // 这意味着，如果您已经完成了10次迭代，则此函数返回矩阵以将点云从迭代10转换为11。
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;

                // 函数getFinalTransformation（）返回在迭代过程中完成的刚性矩阵转换（此处为1次迭代）。
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }

            //这不是我们想要的。 如果我们将最后一个矩阵与新矩阵相乘，那么结果就是从开始到当前迭代的转换矩阵。
        }
        next_iteration = false;
    }
    return (0);
}