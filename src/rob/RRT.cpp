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

#include "geomSolution.cpp"

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;


#define MAXTIME 60.

int main(int argc, char** argv) 
{

    std::string dataFile ("rrt.csv");
    char toDelete[] {"rrt.csv"};
    std::ofstream mydata;
    std::remove(toDelete);
    mydata.open(dataFile);
    mydata << "time,distance,extend,trial,eps,steps" << "\n";
    mydata.close();

    mydata.open(dataFile, std::ios_base::app);
    const std::string wcFile = "../resources/Project_WorkCell_Obstacle/Scene.wc.xml";
    const std::string deviceName = "UR-6-85-5-A";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initial reading of the scene
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    State state = wc->getDefaultState();

    // Setup device
    rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }
    // Set the gripper to the adequate initial position
    Device::Ptr gripper = wc->findDevice("WSG50");
    if (gripper == NULL) {
        std::cerr << "Device: gripper WSG50 not found!" << std::endl;
        return 0;
    }
    gripper->setQ(rw::math::Q(1,0.0455), state);


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Read the frames of the objects.
    Frame *bottle_frame = wc->findFrame("BottleLateral");
    Frame *bottleGrasp_frame = wc->findFrame("Bottle");
    Frame *bottle_approx = wc->findFrame("BottleApprox");
    Frame *objective_frame = wc->findFrame("Objective");
    Frame *objectiveUp = wc->findFrame("ObjectiveUp");
    Frame *bottleUp = wc->findFrame("BottleUp");
    Frame * RobBase = wc->findFrame("URReference");
    Frame * Grasp = wc->findFrame("GraspTCP");
    Frame * Tool = wc->findFrame("UR-6-85-5-A.TCP");
    rw::math::Transform3D<> TTool = rw::kinematics::Kinematics::frameTframe(Grasp, Tool, state);
    // Check if the frames were found
    if((objective_frame==nullptr) || (bottle_frame==nullptr) || (bottle_approx==nullptr) || (bottleGrasp_frame==nullptr) || (bottleUp==nullptr) || (objectiveUp==nullptr))
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Pre-obtain the poses of the robot
    rw::math::Q initialPose, approxPose, pickPose, endPose;

    // Initial pose
    initialPose = device->getQ(wc->getDefaultState());

    // Put the Robot in the approx pick position
    rw::math::Transform3D<> TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottle_approx, state);
    rw::math::Transform3D<> targetAt = TR * TTool;
    rw::invkin::ClosedFormIKSolverUR::Ptr solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    std::vector<rw::math::Q> solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the pickup:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol))
        {
            //std::cout << "Found a pose with no collision!" << std::endl;
            approxPose = sol;
            break;
        }
    }

    // Put the Robot in the pick position
    TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottleUp, state);
    targetAt = TR * TTool;
    solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the pickup:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol))
        {
            //std::cout << "Found a pose with no collision!" << std::endl;
            pickPose = sol;
            break;
        }
    }

    // Put the Robot in the objective position
    TR = rw::kinematics::Kinematics::frameTframe(RobBase, objectiveUp, state);
    targetAt = TR * TTool;
    solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the placing:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol))
        {
            //std::cout << "Found a pose with no collision in the end frame!" << std::endl;
            endPose = sol;
            break;
        }
    }

    // Obtain the interpolator and subpath for the approx position
    int n_poses {10};
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator (approxPose, endPose, 1);

    std::cout << " - The found poses are: \n - " << pickPose << "\n - " << endPose << std::endl;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The whole movement
    for (double extend = 0.1; extend <= 1.0 ; extend+=0.1)
    {
        for(int trial = 0; trial < 40; trial++) 
        {
            std::cout << " ----------------------------- Extension: " << extend << "  ----- Trial: " << trial << "   --------------------------\n";
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Set some variables
            rw::math::Math::seed(trial);

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Set State and Workspace, and initial states and frames

            // Initiate State and WorkCell
            Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
            rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
            State state = wc->getDefaultState();

            // Setup device
            rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(deviceName);
            if (device == NULL) {
                std::cerr << "Device: " << deviceName << " not found!" << std::endl;
                return 0;
            }


            // Set the gripper to the adequate initial position
            Device::Ptr gripper = wc->findDevice("WSG50");
            if (gripper == NULL) {
                std::cerr << "Device: gripper WSG50 not found!" << std::endl;
                return 0;
            }
            gripper->setQ(rw::math::Q(1,0.05), state);


            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Setting up the solver

            // Parameters for the planner
            rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);
            rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),constraint.getQConstraintPtr());
            rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
            rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);
            rw::common::Timer t;
    
            // Path 1
            rw::trajectory::QPath path1;
            t.resetAndResume();
            planner->query(initialPose, approxPose,path1,MAXTIME);
            t.pause();
            double distance = 0;

            std::cout << "Path1 of length " << path1.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            std::cout << "Variables to introduce: "<< t.getTime() << "," << path1.size() << "," << extend << "," << trial << "\n";
            mydata << t.getTime() << "," << path1.size() << "," << extend << "," << trial << "\n";

            // Perform the gripping
            device->setQ(pickPose, state);
            rw::kinematics::Kinematics::gripFrame(bottleGrasp_frame, Grasp, state);

            // Path 2
            rw::trajectory::QPath path2;
            t.resetAndResume();
            planner->query(pickPose, endPose,path1,MAXTIME);
            t.pause();

            std::cout << "Path2 of length " << path2.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            std::cout << "Variables to introduce: "<< t.getTime() << "," << path2.size() << "," << extend << "," << trial << "\n";
            mydata << t.getTime() << "," << path2.size() << "," << extend << "," << trial << "\n";
            
            // visualize them
            double time = 0.0;
    		rw::trajectory::TimedStatePath tStatePath;
		    for(uint i=0; i<path2.size(); i+=1)
            {
        	    device->setQ(path2.at(i), state);
        	    tStatePath.push_back(rw::trajectory::TimedState(time,state));
		        time += 0.01;
    		}   

    		rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "RRT.rwplay");

        }
    }
    mydata.close();
	return 0;
}
