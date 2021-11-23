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
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "stereoAid.cpp"

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;
typedef rw::sensor::Camera Camera;

const int width {200}, height {200};

int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize
    const std::string wcFile = "../resources/Project_WorkCell/Scene.wc.xml";
    std::cout << "Trying to use workcell " << wcFile <<  std::endl;
    std::string deviceName = "UR-6-85-5-A";


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set some variables
    rwlibs::simulation::SimulatedCamera::Ptr leftCamera, rightCamera;
    cv::Mat left(cv::Size(100,100), CV_64FC1, cv::Scalar(0));
    cv::Mat right(cv::Size(100,100), CV_64FC1, cv::Scalar(0));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set State and Workspace, and initial states and frames

    // Initiate State and WorkCell
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();

    // Setup device
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL)
    {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }


    return 0;
}