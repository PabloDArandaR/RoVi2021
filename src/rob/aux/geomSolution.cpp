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
//#include <rw/trajectory/RampInterpolator.hpp>
#include <rw/trajectory/BlendedTrajectory.hpp>
//#include <rw/trajectory.hpp>
#include <rw/math.hpp>
#include <rw/invkin.hpp>


typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;

rw::math::Q findSolution(std::vector<rw::math::Q> solutionVector)
{
    if(solutionVector.size() > 0)
    {    
        for (rw::math::Q sol: solutionVector)
        {
            if ((sol[2] < 0) && (sol[3] > -3) && (sol[3] < -0.5) && (sol[5] <= 0))
            {
                return sol;
            }
        }
        std::cerr << " [ERROR] The solutions find weren't suitable.\n";
        return 1;
    }
    else
    {
        std::cerr << " [ERROR] No solutions found." << std::endl;
        return 1;
    }
}

bool checkCollisions(Device::Ptr device, const State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
	State testState;
	rw::proximity::CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		//std::cerr << "Configuration in collision: " << q << std::endl;
		//std::cerr << "Colliding frames: " << std::endl;
		rw::kinematics::FramePairSet fps = data.collidingFrames;
		for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			//std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
		}
		return false;
	}
    return true;
}