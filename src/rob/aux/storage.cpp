#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <stdlib.h> 
#include <rw/rw.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/Timer.hpp>
#include <rw/proximity/ProximityStrategy.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTNode.hpp>
#include <rw/trajectory.hpp>

void storePathRRT(std::vector<rw::math::Q> path1, std::vector<rw::math::Q> path2, std::string wcFile, std::string deviceName, Frame * Grasp, Frame * bottle, std::string fileName)
{
    ///////////////////////////////////////////////////////////////////////////
    // Initialized the scene
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();
    // Setup device
    Device::Ptr device = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
    }
    
    // Set the gripper to the adequate initial position
    Device::Ptr gripper = wc->findDevice("WSG50");
    if (gripper == NULL) {
        std::cerr << "Device: gripper WSG50 not found!" << std::endl;
    }
    gripper->setQ(rw::math::Q(1,0.0455), state);

    ///////////////////////////////////////////////////////////////////////////
    // Store the Path
    double time = 0.0;
    rw::trajectory::TimedStatePath tStatePath;
    for(uint i=0; i<path1.size(); i+=1)
    {
        device->setQ(path1.at(i), state);
        tStatePath.push_back(rw::trajectory::TimedState(time,state));
        time += 0.1;
    }
    rw::kinematics::Kinematics::gripFrame(bottle, Grasp, state);
    for(uint i=0; i<path2.size(); i+=1)
    {
        device->setQ(path2.at(i), state);
        tStatePath.push_back(rw::trajectory::TimedState(time,state));
        time += 0.1;
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, fileName);
}
