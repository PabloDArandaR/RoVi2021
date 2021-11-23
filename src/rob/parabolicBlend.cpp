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
#include "geomSolution.cpp"

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;

int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Basic variables
    double time_ {0};
    double precision {0.1};
    double approxDuration {3.0};
    double linearDuration {3.0};
    double duration {0.0};
    double tau {1.0};
    std::string pathName {"blend.csv"};
    std::string pointName {"pointBlend.csv"};
    const std::string wcFile = "../resources/Project_WorkCell/Scene.wc.xml";
    const std::string deviceName = "UR-6-85-5-A";
    std::fstream data, point;
    int nApprox {100}, nLinear{100}, nBlend{100};

    // File cleaning
    char toDelete1[] = "blend.csv";
    char toDelete2[] = "pointBlend.csv";
    std::remove(toDelete1);
    std::remove(toDelete2);

    // Initiate State and WorkCell
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;
    Cell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();

    // Setup device
    Device::Ptr gripper= wc->findDevice("WSG50");
    rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if (device == NULL) {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }
    if (gripper == NULL) {
        std::cerr << "Device: gripper WSG50 not found!" << std::endl;
        return 0;
    }
    gripper->setQ(rw::math::Q(1,0.055), state);

    // Main variables regarding the paths
    rw::math::Q initial =  device->getQ(state);
    std::vector<rw::math::Q> approximation1, approximation2, wholePath;
    std::vector<double> timeVector;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize the frame variables

    // Read the frames of the objects.
    Frame *bottle_frame = wc->findFrame("BottleLateral");
    Frame *bottleGrasp_frame = wc->findFrame("Bottle");
    Frame *bottle_approx = wc->findFrame("BottleApprox");
    Frame *objective_frame = wc->findFrame("Objective");
    Frame *objectiveApprox_frame = wc->findFrame("ObjectiveApprox");
    Frame * RobBase = wc->findFrame("URReference");
    Frame * Grasp = wc->findFrame("GraspTCP");
    Frame * Tool = wc->findFrame("UR-6-85-5-A.TCP");

    if ((bottle_frame == nullptr) || (RobBase == nullptr) || (Grasp == nullptr) || (Tool == nullptr) || (objective_frame == nullptr))
    {
        std::cerr << " [ERROR] At least one of the frames was not found" << std::endl;
        return 1;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Generating the path to the points

    rw::math::Transform3D<> TTool = rw::kinematics::Kinematics::frameTframe(Grasp, Tool, state);

    // Approximation 1:
    approximation1.push_back(initial);
    {
        // Approximation point
        rw::math::Transform3D<> TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottle_approx, state);
        rw::math::Transform3D<> targetAt = TR * TTool;
        rw::invkin::ClosedFormIKSolverUR::Ptr solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
        std::vector<rw::math::Q> solutions = solverUR->solve(targetAt, state);
        approximation1.push_back(findSolution(solutions));

        // Grasping point
        TR = rw::kinematics::Kinematics::frameTframe(RobBase, bottle_frame, state);
        targetAt = TR * TTool;
        solutions = solverUR->solve(targetAt, state);
        approximation1.push_back(findSolution(solutions));
    }

    std::vector<std::string> frames = {"CrossPoint1", "CrossPoint2", "CrossPoint3", "CrossPoint4", "CrossPoint5", "CrossPoint6"};
    wholePath.push_back(approximation1[approximation1.size()-1]);
    for (std::string frameName: frames)
    {
        Frame * PiF = wc->findFrame(frameName);
        if (PiF == nullptr)
        {
            std::cerr << " [ERROR] Frame " << frameName << " not found.\n";
        }
        rw::math::Transform3D<> TR = rw::kinematics::Kinematics::frameTframe(RobBase, PiF, state);
        rw::math::Transform3D<> targetAt = TR * TTool;
        rw::invkin::ClosedFormIKSolverUR::Ptr solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
        std::vector<rw::math::Q> solutions = solverUR->solve(targetAt, state);
        wholePath.push_back(findSolution(solutions));
    }

    approximation2.push_back(wholePath[wholePath.size()-1]);
    {   
        rw::math::Transform3D<> TR = rw::kinematics::Kinematics::frameTframe(RobBase, objectiveApprox_frame, state);
        rw::math::Transform3D<> targetAt = TR * TTool;
        rw::invkin::ClosedFormIKSolverUR::Ptr solverUR = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR (device, state));
        std::vector<rw::math::Q> solutions = solverUR->solve(targetAt, state);
        approximation2.push_back(findSolution(solutions));

        // Grasping point
        TR = rw::kinematics::Kinematics::frameTframe(RobBase, objective_frame, state);
        targetAt = TR * TTool;
        solutions = solverUR->solve(targetAt, state);
        approximation2.push_back(findSolution(solutions));
    }


    data.open(pathName, std::ios::trunc);
    data.close();
    data.open(pathName, std::ios::app);
    data << "joint0,joint1,joint2,joint3,joint4,joint5,time\n";
    double timePlot {0.0};

    point.open(pointName, std::ios::trunc);
    point.close();
    point.open(pointName, std::ios::app);
    point << "joint0,joint1,joint2,joint3,joint4,joint5,time\n";


    // Obtain the path and write it
    time_ = 0.0;
    rw::trajectory::TimedStatePath tStatePath;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // First approximation

    double begin {0.0};

    for (uint i = 0; i < approximation1.size()-1; i++)
    {
        begin = time_;
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(approximation1[i], approximation1[i+1], approxDuration);
        precision = approxDuration/nApprox;
        for (int j = 0; j < nApprox; j++)
        {
            rw::math::Q newPose = interpolator.x(time_ - begin);
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;

            time_ += precision;
        }
    }
    device->setQ(approximation1[approximation1.size() - 1], state);
    rw::kinematics::Kinematics::gripFrame(bottleGrasp_frame, Grasp, state);
    tStatePath.push_back(rw::trajectory::TimedState(time_, state));
    time_ += precision;

    for (uint i = 0; i < approximation1.size(); i++)
    {
        for (int k = 0; k < 6; k++)
        {
            point << approximation1[i][k] << ",";
        }
        point << timePlot << std::endl;
        timePlot += approxDuration;
    }
    std::cout << "The time in the end of the approximation 1 is " << time_ << std::endl;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Blend part
    begin = 0.0;
    for (uint i = 0; i < wholePath.size()-2; i ++)
    {
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator1(wholePath[i], wholePath[i+1], linearDuration);
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator2(wholePath[i+1], wholePath[i+2], linearDuration);
        rw::trajectory::ParabolicBlend<rw::math::Q> interpolatorB(interpolator1, interpolator2, tau);
        std::cout << "The points involved in this iteration are:\n    -" << wholePath[i] << "\n    -" << wholePath[i+1] << "\n    -" << wholePath[i+2] << std::endl;
        
        // Linear part
        duration = (linearDuration - begin - tau);
        precision = duration/nLinear;
        double initialTime = time_;
        for (int j = 0; j < nLinear; j++)
        {
            rw::math::Q newPose = interpolator1.x(time_-initialTime + begin);
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;
            
            time_ += precision;
        }

        std::cout << "The time in the end of the linear part in the iteriation " << i << " is: " << time_ << std::endl;

        // Blend part
        initialTime = time_;
        duration = tau*2;
        precision = duration/nBlend;
        for (int j = 0; j < nBlend; j++)
        {
            rw::math::Q newPose = interpolatorB.x(time_-initialTime);
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;
            
            time_ += precision;
        }
        begin = tau;

        std::cout << "The time in the end of the parabolic part in the iteriation " << i << " is: " << time_ << std::endl;
    }


    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator1(wholePath[wholePath.size()-2], wholePath[wholePath.size()-1], linearDuration);
    {
        precision = (linearDuration - begin)/nLinear;
        double initialTime = time_;
        for (int j = 0; j < nLinear; j++)
        {
            rw::math::Q newPose = interpolator1.x(time_-initialTime + begin);
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;

            time_ += precision;
        }
    }

    for (uint i = 1; i < wholePath.size() - 1; i++)
    {
        for (int k = 0; k < 6; k++)
        {
            point << wholePath[i][k] << ",";
        }
        point << timePlot << std::endl;
        timePlot += linearDuration;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Second approximation

    for (uint i = 0; i < approximation2.size()-1; i++)
    {
        begin = time_;
        std::cout << " The points of the interpolator are: \n     -" << approximation2[i] << "     -" << approximation2[i+1] << std::endl;
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(approximation2[i], approximation2[i+1], approxDuration);
        precision = approxDuration/nApprox;
        for (int j = 0; j < nApprox; j++)
        {
            rw::math::Q newPose = interpolator.x(time_ - begin);
            //std::cout << "The new pose is: \n" << newPose << std::endl;
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;

            time_ += precision;
        }
    }

    for (uint i = 0; i < approximation2.size(); i++)
    {
        for (int k = 0; k < 6; k++)
        {
            point << approximation2[i][k] << ",";
        }
        point << timePlot << std::endl;
        timePlot += approxDuration;
    }

    data.close();
    point.close();
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "pathBlend.rwplay");

    std::cout << "Exiting...\n" << std::endl;

    return 0;
}