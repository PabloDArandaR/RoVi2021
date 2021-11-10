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

#include "interpol.cpp"
#include "parabolicBlend.cpp"

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;

int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Basic variables
    const std::string wcFile = "../resources/Project_WorkCell/Scene.wc.xml";
    const std::string deviceName = "UR-6-85-5-A";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;
    double precision {0.001};
    std::string pathFileName {"blendPath.csv"}, pointsName {"pointsBlend.csv"};
    std::fstream pathFile, pointFile;

    // Prepare the files
    pathFile.open(pathFileName, std::ios::trunc);
    pathFile << "joint0,joint1,joint2,joint3,joint4,joint5,time\n";
    pathFile.close();

    // Initiate State and WorkCell
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();

    // Setup device
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Input parameters for the interpolator

    // Points to go through
    rw::math::Q initial =  device->getQ(state);
    rw::math::Q P1(6,-0.534, -1.037, -1.304, -3.112, 0.099, 0);
    rw::math::Q end(6, 0, 0, 0, 0, 0, 0);
    std::vector<rw::math::Q> totalPath{initial, P1, end};
    std::vector<double> totalTime{0,5,10};

    // Add points to the path

    std::cout << "Value of P1:  " << P1 << std::endl;

    // Other input parameters
    std::vector<double> timePoints {0, 5, 10};
    std::vector<double> blendAcceleration {1, 1, 1};
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Read the frames of the objects.
    Frame *tool_frame = wc->findFrame("Tool");
    Frame *bottle_frame = wc->findFrame("Bottle");

    // Obtain the interpolator
    parabolicBlendWithViapoints blendInterpolator(initial, end, totalPath, blendAcceleration, timePoints);
    std::cout << "[NOTE] Interpolator generated." << std::endl;
    
    // Generate the path
    std::vector<double> time_= timeVector(totalTime[totalTime.size()-1], precision);
    int nPoints {0};
    std::vector<rw::math::Q> path;
    rw::trajectory::TimedStatePath tStatePath;
    pathFile.open(pathFileName, std::ios::app);
    for (uint i = 0; i < time_.size(); i++)
    {
        rw::math::Q newPose = blendInterpolator.x(time_[i]);
        path.push_back(newPose);
        std::cout << "    [NOTE] The new Pose is:              " << newPose << std::endl;
        device->setQ(newPose, state);
        tStatePath.push_back(rw::trajectory::TimedState(time_[i], state));
        for (int i = 0; i < 6; i++)
        {
            pathFile << newPose[i]<<",";
        }
        pathFile << time_[i] << std::endl;
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "pathBlend.rwplay");
    pathFile.close();

    return 0;
}