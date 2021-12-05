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

#include "aux/geomSolution.cpp"
#include "aux/pruning.cpp"
#include "aux/storage.cpp"

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;

#define MIN_EXTEND 0.1 
#define MAX_EXTEND 1.
#define N_TRIALS 40 
#define MAXTIME 60.

int main(int argc, char** argv) 
{

    // std::string dataFile ("rrt.csv");
    char dataFile[] {"../results/rrt.csv"};
    std::ofstream mydata;
    std::remove(dataFile);
    mydata.open(dataFile);
    mydata << "time1,time2,time1prun,time2prun,size1,size2,size1prun,size2prun,extend,pickPose" << "\n";

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
    Frame *bottleLateral = wc->findFrame("BottleLateral");
    Frame *bottle = wc->findFrame("BottleLateral2");
    Frame *bottleUp = wc->findFrame("BottleUp");
    Frame *objective = wc->findFrame("Objective");
    Frame *objectiveUp = wc->findFrame("ObjectiveUp");
    Frame * RobBase = wc->findFrame("URReference");
    Frame * Grasp = wc->findFrame("GraspTCP");
    Frame * Tool = wc->findFrame("UR-6-85-5-A.TCP");
    rw::math::Transform3D<> TTool = rw::kinematics::Kinematics::frameTframe(Grasp, Tool, state);
    // Check if the frames were found
    if((objective==nullptr) || (objectiveUp==nullptr) || (bottle==nullptr) || (bottleUp==nullptr) || (bottleLateral==nullptr))
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Pre-obtain the poses of the robot
    rw::math::Q initialPose, pickPose1, pickPose2, pickPose3, endPose1, endPose2;

    // Initial pose
    initialPose = device->getQ(wc->getDefaultState());

    // Put the Robot in the pick position
    rw::math::Transform3D<> TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottleUp, state);
    rw::math::Transform3D<> targetAt = TR * TTool;
    rw::invkin::ClosedFormIKSolverUR::Ptr solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    std::vector<rw::math::Q> solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the pickup 1:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
        {
            pickPose1 = sol;
            break;
        }
    }
    if (pickPose1.size() < 6)
    {
        for (rw::math::Q sol: solutions)
        {
            std::cout << "     - Solution:" << sol << std::endl;
            if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
            {
                pickPose1 = sol;
                break;
            }
        }
    }

    TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottle, state);
    targetAt = TR * TTool;
    gripper->setQ(rw::math::Q(1,0.0455), state);
    solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the pickup 2:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
        {
            pickPose2 = sol;
            break;
        }
    }
    if (pickPose2.size() < 6)
    {
        for (rw::math::Q sol: solutions)
        {
            std::cout << "     - Solution:" << sol << std::endl;
            if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
            {
                pickPose2 = sol;
                break;
            }
        }
    }

    TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottleLateral, state);
    targetAt = TR * TTool;
    gripper->setQ(rw::math::Q(1,0.0455), state);
    solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the pickup 3:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
        {
            pickPose3 = sol;
            break;
        }
    }
    if (pickPose3.size() < 6)
    {
        for (rw::math::Q sol: solutions)
        {
            std::cout << "     - Solution:" << sol << std::endl;
            if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
            {
                pickPose3 = sol;
                break;
            }
        }
    }

    // Put the Robot in the objective position
    TR = rw::kinematics::Kinematics::frameTframe(RobBase, objectiveUp, state);
    targetAt = TR * TTool;
    gripper->setQ(rw::math::Q(1,0.0455), state);
    solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the placing 1:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
        {
            endPose1 = sol;
            break;
        }
    }
    if (endPose1.size() < 6)
    {
        for (rw::math::Q sol: solutions)
        {
            std::cout << "     - Solution:" << sol << std::endl;
            if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
            {
                endPose1 = sol;
                break;
            }
        }
    }

    TR = rw::kinematics::Kinematics::frameTframe(RobBase, objective, state);
    targetAt = TR * TTool;
    gripper->setQ(rw::math::Q(1,0.0455), state);
    solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
    solutions = solverUR->solve(targetAt, state);
    std::cout << "  -Solutions for the placing 2:\n";
    for (rw::math::Q sol: solutions)
    {
        std::cout << "     - Solution:" << sol << std::endl;
        if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
        {
            endPose2 = sol;
            break;
        }
    }
    if (endPose2.size() < 6)
    {
        for (rw::math::Q sol: solutions)
        {
            std::cout << "     - Solution:" << sol << std::endl;
            if (checkCollisions(device, state, detector, sol) && (sol[3] > -2.9) && (sol[3] < 0.8))
            {
                endPose2 = sol;
                break;
            }
        }
    }

    std::cout << " - Solutions found:\n     -pickPose1: "<< pickPose1 << "\n     -pickPose2: "<< pickPose2 << "\n     -pickPose3: "<< pickPose3 << "\n     -endPose1: "<< endPose1 << "\n     -endPose2: "<< endPose2 << std::endl;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The whole movement

    // First pick pose
    for (double extend = MIN_EXTEND; extend <= MAX_EXTEND ; extend+=0.1)
    {
        for(int trial = 0; trial < N_TRIALS; trial++) 
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
            double t1, t2, t1_prunn, t2_prunn;
            rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);
            rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),constraint.getQConstraintPtr());
            rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
            rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);
            rw::common::Timer t;
    
            // Path 1
            rw::trajectory::QPath path1;
            t.resetAndResume();
            planner->query(initialPose, pickPose1, path1, MAXTIME);
            t.pause();
            std::cout << "Path1 of length " << path1.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            t1 = t.getTime();

            // Perform the gripping
            device->setQ(pickPose1, state);
            rw::kinematics::Kinematics::gripFrame(bottle, Grasp, state);

            // Path 2
            rw::trajectory::QPath path2;
            t.resetAndResume();
            planner->query(pickPose1, endPose1, path2, MAXTIME);
            t.pause();
            std::cout << "Path2 of length " << path2.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            t2 = t.getTime();
            

            // Pruning
            rw::trajectory::QPath path2pruned, path1pruned;

            t.resetAndResume();
            pathPruning(deviceName, path1, &path1pruned, wc);
            t.pause();
            std::cout << "Path1pruned of length " << path1pruned.size() << " obtained in " << t.getTime() << " seconds." << std::endl;
            t1_prunn = t.getTime();

            t.resetAndResume();
            pathPruning(deviceName, path2, &path2pruned, wc);
            t.pause();
            std::cout << "Path2pruned of length " << path2pruned.size() << " obtained in " << t.getTime() << " seconds." << std::endl;
            t2_prunn = t.getTime();

            // Store information:
            // mydata << "time1,time2,time1prun,time2prun,size1,size2,size1prun,size2prun,extend,trial" << "\n";
            mydata << t1<<","<<t2<<","<<t1_prunn<<","<<t2_prunn<<","<<path1.size()<<","<<path2.size()<<","<<path1pruned.size()<<","<<path2pruned.size()<<","<<extend<<",1"<<std::endl;

            // visualize them
            if ((extend == 0.1) && (trial == 0))
            {
                storePathRRT(path1pruned, path2pruned, wcFile, deviceName, Grasp, bottle, "../results/replayRRT1.rwplay");
            }

            
        }
    }
    
    // Second pick pose
    for (double extend = MIN_EXTEND; extend <= MAX_EXTEND ; extend+=0.1)
    {
        for(int trial = 0; trial < N_TRIALS; trial++) 
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
            double t1, t2, t1_prunn, t2_prunn;
            rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);
            rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),constraint.getQConstraintPtr());
            rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
            rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);
            rw::common::Timer t;
    
            // Path 1
            rw::trajectory::QPath path1;
            t.resetAndResume();
            planner->query(initialPose, pickPose2, path1, MAXTIME);
            t.pause();
            std::cout << "Path1 of length " << path1.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            t1 = t.getTime();

            // Perform the gripping
            device->setQ(pickPose2, state);
            rw::kinematics::Kinematics::gripFrame(bottle, Grasp, state);

            // Path 2
            rw::trajectory::QPath path2;
            t.resetAndResume();
            planner->query(pickPose2, endPose2, path2, MAXTIME);
            t.pause();
            std::cout << "Path2 of length " << path2.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            t2 = t.getTime();
            

            // Pruning
            rw::trajectory::QPath path2pruned, path1pruned;

            t.resetAndResume();
            pathPruning(deviceName, path1, &path1pruned, wc);
            t.pause();
            std::cout << "Path1pruned of length " << path1pruned.size() << " obtained in " << t.getTime() << " seconds." << std::endl;
            t1_prunn = t.getTime();

            t.resetAndResume();
            pathPruning(deviceName, path2, &path2pruned, wc);
            t.pause();
            std::cout << "Path2pruned of length " << path2pruned.size() << " obtained in " << t.getTime() << " seconds." << std::endl;
            t2_prunn = t.getTime();

            // Store information:
            // mydata << "time1,time2,time1prun,time2prun,size1,size2,size1prun,size2prun,extend,trial" << "\n";
            mydata << t1<<","<<t2<<","<<t1_prunn<<","<<t2_prunn<<","<<path1.size()<<","<<path2.size()<<","<<path1pruned.size()<<","<<path2pruned.size()<<","<<extend<<",2"<<std::endl;

            // visualize them
            if ((extend == 0.1) && (trial == 0))
            {
                storePathRRT(path1pruned, path2pruned, wcFile, deviceName, Grasp, bottle, "../results/replayRRT2.rwplay");
            }
        }
    }
    
    // Third pick pose
    for (double extend = MIN_EXTEND; extend <= MAX_EXTEND; extend+=0.1)
    {
        for(int trial = 0; trial < N_TRIALS; trial++) 
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
            double t1, t2, t1_prunn, t2_prunn;
            rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);
            rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),constraint.getQConstraintPtr());
            rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
            rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);
            rw::common::Timer t;
    
            // Path 1
            rw::trajectory::QPath path1;
            t.resetAndResume();
            planner->query(initialPose, pickPose3, path1, MAXTIME);
            t.pause();
            std::cout << "Path1 of length " << path1.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            t1 = t.getTime();

            // Perform the gripping
            device->setQ(pickPose3, state);
            rw::kinematics::Kinematics::gripFrame(bottle, Grasp, state);

            // Path 2
            rw::trajectory::QPath path2;
            t.resetAndResume();
            planner->query(pickPose3, endPose2, path2, MAXTIME);
            t.pause();
            std::cout << "Path2 of length " << path2.size() << " found in " << t.getTime() << " seconds." << std::endl;
            if (t.getTime() >= MAXTIME)
            {
                std::cout << "Notice: max time of " << MAXTIME << " seconds reached." << std::endl;
            }
            t2 = t.getTime();
            

            // Pruning
            rw::trajectory::QPath path2pruned, path1pruned;

            t.resetAndResume();
            pathPruning(deviceName, path1, &path1pruned, wc);
            t.pause();
            std::cout << "Path1pruned of length " << path1pruned.size() << " obtained in " << t.getTime() << " seconds." << std::endl;
            t1_prunn = t.getTime();

            t.resetAndResume();
            pathPruning(deviceName, path2, &path2pruned, wc);
            t.pause();
            std::cout << "Path2pruned of length " << path2pruned.size() << " obtained in " << t.getTime() << " seconds." << std::endl;
            t2_prunn = t.getTime();

            // Store information:
            // mydata << "time1,time2,time1prun,time2prun,size1,size2,size1prun,size2prun,extend,trial" << "\n";
            mydata << t1<<","<<t2<<","<<t1_prunn<<","<<t2_prunn<<","<<path1.size()<<","<<path2.size()<<","<<path1pruned.size()<<","<<path2pruned.size()<<","<<extend<<","<<"3"<<std::endl;

            // visualize them
            if ((extend == 0.1) && (trial == 0))
            {
                storePathRRT(path1pruned, path2pruned, wcFile, deviceName, Grasp, bottle, "../results/replayRRT3.rwplay");
            }
        }
    }

    mydata.close();
	return 0;
}