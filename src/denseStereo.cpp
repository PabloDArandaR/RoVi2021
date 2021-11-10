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
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;
typedef rw::sensor::Camera Camera;

int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set random seed
    rw::math::Math::seed(100);
    const std::string wcFile = "../resources/Kr16WallWorkCell/Scene.wc.xml";
    const std::string deviceName = "KukaKr16";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set some variables
    rwlibs::simulation::SimulatedCamera::Ptr leftCamera, rightCamera;
    cv::Mat left, right;

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
    Frame *circle_frame = wc->findFrame("Circle");
    Frame *square_frame = wc->findFrame("Square");
    Frame *leftCamera_frame = wc->findFrame("Left_Camera");
    Frame *rightCamera_frame = wc->findFrame("Right_Camera");

    //rwRobWorkStudioApp app ("simulation");

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Dense stereo implementation

    cv::Mat disp = right - left;


}       