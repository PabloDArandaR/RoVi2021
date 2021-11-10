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

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;
typedef rw::sensor::Camera Camera;

const int width {200}, height {200};

cv::Mat defineQ(int img_width, int img_height)
{
    // Initialize certain variables:
    float f {500}, Tx {-100};
    float I_w_term {(float)-img_width/(float)2.0};
    float I_h_term {(float)-img_height/(float)2.0};
    float translation_X {(float)-1/Tx};
    float data [16] = {1, 0, 0, I_w_term, 0, 1, 0, I_h_term, 0, 0, 0, f, 0, 0, translation_X, 0};

    // Define Q matrix as per document
    cv::Mat matQ = cv::Mat(4,4, CV_32F, data);

    return matQ;
}

cv::Mat DisparityBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    cv::Mat result;

    // Compute disparity with cv::StereoBM
    cv::Ptr<cv::StereoBM> bM = cv::StereoBM::create(nDisparities, BlockSize);
    bM->compute(imgL, imgR, result);

    return result;
}

cv::Mat DisparitySGBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    cv::Mat result;

    // Compute disparity with cv::StereoSGBM
    cv::Ptr<cv::StereoSGBM> bM = cv::StereoSGBM::create(nDisparities, BlockSize);
    bM->compute(imgL, imgR, result);

    return result;
}

cv::Mat normDisparity(cv::Mat disp)
{
    // Normalize matrix to unsigned integers between 0-255 needed for visualization
    for (int i = 0 ; i < disp.rows; i++)
    {
        uchar * pixel = disp.ptr(i);
        for (int j = 0; j < disp.cols; j++)
        {
            pixel[j] = (uchar)((int)pixel[j] + 127);
        }
    }

    return disp;
}

cv::Mat reproject3D(cv::Mat disp, cv::Mat Q)
{
    cv::Mat points;
    
    // Reproject disparity image to 3D
    cv::reprojectImageTo3D(disp, points, Q, true);
    return points;
}

void savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        for (size_t j = 0; j < points.cols; j++) {
          if
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}

int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize
    const std::string wcFile = "../resources/Project_WorkCell/Scene.wc.xml";
    const std::string deviceName = "UR-6-85-5-A";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set some variables
    rwlibs::simulation::SimulatedCamera::Ptr leftCamera, rightCamera;
    cv::Mat left(cv::Size(100,100), CV_64FC1, cv::Scalar(0)), right(cv::Size(100,100), CV_64FC1, cv::Scalar(0));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set State and Workspace, and initial states and frames

    // Initiate State and WorkCell
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();

    // Setup device
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }

    // Read the frames of the objects.
    Frame *tool_frame = wc->findFrame("Tool");
    Frame *bottle_frame = wc->findFrame("Bottle");
    Frame *leftCamera_frame = wc->findFrame("Left_Camera");
    Frame *rightCamera_frame = wc->findFrame("Right_Camera");

    //rwRobWorkStudioApp app ("simulation");

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Dense stereo implementation 

    // Rectify and undistort images (if necessary)
    
        //Since the cameras are parallel, the images are rectified already

    // Compute disparity map
    cv::cvtColor(right, right, cv::COLOR_RGB2GRAY);
    cv::cvtColor(left, left, cv::COLOR_RGB2GRAY);

    cv::Mat disp = DisparitySGBM(left, right, 21, 10);

    // Compute point cloud
    
    cv::Mat points = reproject3D(disp, defineQ(width, height));


    // Filter/segment the point cloud

    // Global pose estimation to find the object pose

    // Local pose estimation to find the object pose

    return 0;
}

