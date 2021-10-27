#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <stdlib.h> 
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;

void RRT(int seed, Cell * wc, Frame * origin, Frame * end, Device * robot)
{
    // Set random seed
    rw::math::Math::seed(seed);

    // Generate different poses/configurations for each of the beginning and end points.

    // Optimization of the possible found poses

    // Planner to generate the paths from one point to the other

    // Generate the paths for the given initial/end points

    // Optimization

}       