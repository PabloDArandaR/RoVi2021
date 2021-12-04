#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

#include "aux/stereoAid.cpp"

using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

const float width {640}, height {480};
const std::string filename {"../resources/dense.pcd"};
const float fov {50};
const float Tx{-200}, f {width/tan(M_PI*fov/(2.0*360.0))};

int main(int argc, char* argv[])
{
    std::cout << "[NOTE] Read the images" << std::endl;
    cv::Mat left {cv::imread("../resources/leftImage.png")}, right {cv::imread("../resources/rightImage.png")};
    std::cout << "[NOTE] Images read" << std::endl;
    int method {2}, min_disp{0}, n_disp {0}, blockSize {0}, P1 {0}, P2 {0}, disp12MaxDiff {0}, preFilterCap {0}, uniquenessRatio {0}, speckleWindowSize {0}, speckleRange {0};
    double max_z {0};

    // Argument parser
    if (argc < 11)
    {
        std::cout << "  [ERROR] Not enough input parameters. Arguments to introduce are:\n   -minP_disparities \n   -n_disparities (multiple of 16)\n   -blockSize (odd number)\n   -P1\n   -P2\n   -disp12MaxDiff\n   -preFilterCap\n   -uniquenessRatio\n   -speckleWindowSize\n   -speckleRange\n   -max_z\n";
        return 1;
    }
    else
    {
        min_disp = std::stoi(argv[1]);
        n_disp = std::stoi(argv[2]);
        blockSize = std::stoi(argv[3]);
        P1 = std::stoi(argv[4]);
        P2 = std::stoi(argv[5]);
        disp12MaxDiff = std::stoi(argv[6]);
        preFilterCap = std::stoi(argv[7]);
        uniquenessRatio = std::stoi(argv[8]);
        speckleWindowSize = std::stoi(argv[9]);
        speckleRange = std::stoi(argv[10]);
        max_z = std::stoi(argv[11]);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Disparity calculation

    // Rectify and undistort images (if necessary)
    
    // Compute disparity map
    cv::Mat colors = left;
    cv::cvtColor(right, right, cv::COLOR_RGB2GRAY);
    cv::cvtColor(left, left, cv::COLOR_RGB2GRAY);
    
    cv::Mat disp;
    if (method == 1){
        disp = DisparityBM(left, right, n_disp, blockSize);
    }
    else if (method == 2){
        cv::Ptr<cv::StereoSGBM> bM = cv::StereoSGBM::create(min_disp, n_disp, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
        bM->compute(left, right, disp);
    }
    else{
        std::cout << "[ERROR] Invalid method.\n";
        return 1;
    }
    cv::Mat dispNorm = normDisparity(disp);

    cv::imshow("Disparity",disp);
    cv::imshow("NormDisparity", dispNorm);
    cv::waitKey();


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Calculate PCL

    // Compute point cloud
	auto qMat = defineQ(left.cols, left.rows);
    cv::Mat points = reproject3D(disp, qMat);


    std::cout << " -- Points are :  \n " << points <<  std::endl;
    cv::imshow("Disparity", disp);
    cv::imshow("colors", colors);
    cv::imshow("Points", points);
    cv::waitKey();

    std::cout << "The threshold is:  " << max_z << std::endl;

    pclCloud::Ptr cloud = savePointCloud("../resources/dense.pcd",points, colors, max_z);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // View the PCL

    // Number of channels of images:

    std::cout << "  n_channels dispNorm:  " << dispNorm.channels() << std::endl;
    std::cout << "  n_channels disp:  " << disp.channels() << std::endl;
    std::cout << "  n_channels points:  " << points.channels() << std::endl;
    std::cout << "  n_channels colors:  " << colors.channels() << std::endl;

    //std::cout << " -- Points:   \n" << points << std::endl;

    // Brief setup
    pcl::visualization::PCLVisualizer::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor (0, 0, 0);

    // Add PointCloud to the visualizer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> local_color(cloud, 255, 100, 0);
    viewer->addPointCloud(cloud, local_color, "local");

    // Show PointCloud
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
    }
    viewer->resetStoppedFlag();

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Pose estimation

    return 0;
}