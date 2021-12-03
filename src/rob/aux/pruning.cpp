#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include "interpol.cpp"

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

void pathPruning(std::string deviceName, rw::trajectory::QPath input, rw::trajectory::QPath::Ptr output, WorkCell::Ptr wc)
{
    uint i {0};
    bool collision;
    Device::Ptr device;
    std::vector<rw::math::Q> trajectory;
    rw::kinematics::State state;

    *output = input;
    device = wc->findDevice(deviceName);
    state = wc->getDefaultState();
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    while (i < (output->size() - 2))
    {
        collision = false;
        trajectory = linearInterpolation(output->at(i), output->at(i+2),1,0.01);
        for (uint j = 0; j < trajectory.size(); j++)
        {
            device->setQ(trajectory[j], state);
            if (!checkCollisions(device,state,detector,trajectory[j]))
            {
                collision = true;
                break;
            }
        }
        
        if(collision) i++;
        else 
        {
            output->erase(output->begin()+i+1);
            if (i > 0)
            {
                i--;
            }
        }
    }
}