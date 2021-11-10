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
    double duration {2};
    double time_ {0};
    double precision {0.1};
    std::string pathName {"pointToPoint.csv"};
    std::fstream data;

    // Initiate State and WorkCell
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();

    // Setup device
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }

    // Points to go through
    rw::math::Q initial =  device->getQ(state);
    rw::math::Q P1(6,-0.534, -1.037, -1.304, -3.112, 0.099, 0);
    rw::math::Q P2(6,-0.237, -1.037, -1.949, -3.112, -1.541, 0);
    rw::math::Q P3(6,-0.237, 0.0000, -1.949, -3.112, -1.541, 0);
    rw::math::Q P4(6,-0.237, -1.037, 0.0000, -3.112, -1.541, 0);
    rw::math::Q P5(6,0.0000, -1.037, -1.949, 0.0000, -1.541, 0);
    rw::math::Q P6(6,-0.237, 0.0000, -1.949, -3.112, 0.0000, 0);
    rw::math::Q bottlePoint(6, 1.806, -1.510, -1.822, -1.319, 1.571, 0);
    std::vector<rw::math::Q> wholePath;

    // Add points to the path
    wholePath.push_back(initial); wholePath.push_back(bottlePoint);
    wholePath.push_back(P1); wholePath.push_back(P2);
    wholePath.push_back(P3); wholePath.push_back(P4);
    wholePath.push_back(P5); wholePath.push_back(P6);
    wholePath.push_back(initial);

    // Read the frames of the objects.
    Frame *tool_frame = wc->findFrame("Tool");
    Frame *bottle_frame = wc->findFrame("Bottle");

    // Obtain the path and write it
    std::vector<rw::math::Q> intermediate;
    std::vector<rw::math::Q> path;
    time_ = 0.0;
    rw::trajectory::TimedStatePath tStatePath;
    data.open(pathName, std::ios::trunc);
    data.close();
    data.open(pathName, std::ios::app);
    data << "joint0,joint1,joint2,joint3,joint4,joint5,time\n";
    for (uint i = 1; i < wholePath.size(); i++)
    {
        intermediate = linearInterpolation(wholePath[i-1], wholePath[i], duration, precision);
        for (uint j = 0; j < intermediate.size(); j ++)
        {
            rw::math::Q newPose = intermediate.at(j);
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;

            time_ += precision;
        }
        if ((i == 1) || (i == wholePath.size()-2))
        {
            rw::kinematics::Kinematics::gripFrame(bottle_frame, tool_frame, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            time_ += precision;
        }
    }
    data.close();
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "pathPointToPoint.rwplay");
    std::cout << "Exiting...\n" << std::endl;

    return 0;
}