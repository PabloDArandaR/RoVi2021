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
#include <rw/math/Math.hpp>

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;

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
    Frame *initialFrame0 = wc->findFrame("Bottle");
    Frame *circle_frame = wc->findFrame("Circle");
    Frame *square_frame = wc->findFrame("Square");

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Generate different poses/configurations for each of the beginning and end points.
    rw::math::Rotation3D rotation1(-1,0,0,0,-1,0,0,0,1);
    rw::math::Rotation3D rotation2(0,1,0,1,0,0,0,0,1);
    rw::math::Transformation3D transformation1(rotation1);
    rw::math::Transformation3D transformation2(rotation2);
    std::vector<* Frame> initialFrames;
    initialFrames.push_back(initialFrame0);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Optimization of the possible found poses

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Planner to generate the paths from one point to the other

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Generate the paths for the given initial/end points

    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Optimization


    return 0;
}       