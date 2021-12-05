#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <stdlib.h> 

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/sensor/Camera.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
//#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

int main()
{

    // Declare main variables
    cv::Mat left {cv::imread("../resource/leftImage.png")}, right {cv::imread("../resource/rightImage.png")};

    // Find features
    std::cout << "Version: " << CV_VERSION << std::endl;

    //cv::Ptr<cv::features2D::SIFT> detector = cv::features2D:SIFT::create(100);

    // Match features from one image in the other

    // Find the transformation of one image to the other

    // Should we do the following ones?

        // Compute the disparity map

        // Compute the point cloud

    // Compute the pose of the object

    return 0;
}