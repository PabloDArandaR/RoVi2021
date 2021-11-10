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
#include <rw/trajectory/RampInterpolator.hpp>

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
    double approxDuration {2.0};
    double betweenDuration{2.0};
    std::string pathName {"pointToPoint.csv"};
    std::fstream data;
    int nApprox {100}, nPoint{100}, nBlend{10};
    int tau {1};

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
    std::vector<rw::math::Q> approximation1, approximation2, wholePath;
    

    // Add points to the path
    approximation1.push_back(initial); approximation1.push_back(bottlePoint); approximation1.push_back(initial);

    approximation2.push_back(P5); approximation2.push_back(P6); approximation2.push_back(P5);

    wholePath.push_back(initial); wholePath.push_back(P1); wholePath.push_back(P2); wholePath.push_back(P3); wholePath.push_back(P4); wholePath.push_back(P5); 
    
    std::cout << "The points for approximation 1 are: \n";
    for (int i = 0; i < approximation1.size(); i++)
    {
        std::cout <<"     -" << approximation1[i] << std::endl;
    }
    std::cout << "The points for wholePath are: \n";
    for (int i = 0; i < wholePath.size(); i++)
    {
        std::cout <<"     -" << wholePath[i] << std::endl;
    }
    std::cout << "The points for approximation 2 are: \n";
    for (int i = 0; i < approximation2.size(); i++)
    {
        std::cout <<"     -" << approximation2[i] << std::endl;
    }


    std::cout << "HERE" << std::endl;
    
    // Read the frames of the objects.
    Frame *tool_frame = wc->findFrame("Tool");
    Frame *bottle_frame = wc->findFrame("Bottle");

    // Obtain the path and write it
    time_ = 0.0;
    rw::trajectory::TimedStatePath tStatePath;
    data.open(pathName, std::ios::trunc);
    data.close();
    data.open(pathName, std::ios::app);
    data << "joint0,joint1,joint2,joint3,joint4,joint5,time\n";

    std::cout << "HERE" << std::endl;

    for (uint i = 0; i < approximation1.size()-1; i++)
    {
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(approximation1[i], approximation1[i+1], approxDuration);
        precision = approxDuration/nApprox;
        for (int j = 0; j < nApprox; j++)
        {
            rw::math::Q newPose = interpolator.x(time_);
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
    
    std::cout << "HERE" << std::endl;
    
    std::cout << "the total number of elements in wholePath is: " << wholePath.size() << std::endl;
    for (uint i = 0; i < wholePath.size()-3; i += 1)
    {
        std::cout << "Iteration number: " << i << std::endl;
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator1(wholePath[i], wholePath[i+1], approxDuration);
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator2(wholePath[i+1], wholePath[i+3], approxDuration);
        std::cout << "The points used are: \n    -" << wholePath[i] << "\n    -" << wholePath[i+1] << "\n    -" << wholePath[i+2] << "\n    -" << wholePath[i+3] << std::endl;
        rw::trajectory::ParabolicBlend<rw::math::Q> interpolatorB(interpolator1, interpolator2, tau);
        precision = approxDuration/nPoint;
        double initialTime = time_;
        for (int j = 0; j < nPoint; j++)
        {
            rw::math::Q newPose = interpolator1.x(time_-initialTime);
            device->setQ(newPose, state);
            tStatePath.push_back(rw::trajectory::TimedState(time_,state));
            for (int k = 0; k < 6; k++)
            {
                data << newPose[k] << ",";
            }
            data << time_ << std::endl;
            
            time_ += precision;
        }

        precision = approxDuration/nBlend;
        initialTime = time_;
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

        precision = approxDuration/nPoint;
        initialTime = time_;
        for (int j = 0; j < nPoint; j++)
        {
            rw::math::Q newPose = interpolator2.x(time_-initialTime);
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

    std::cout << "HERE" << std::endl;
    for (uint i = 0; i < approximation2.size(); i++)
    {
        rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(approximation2[0], approximation2[1], approxDuration);
        precision = approxDuration/nApprox;
        for (int j = 0; j < nApprox; j++)
        {
            rw::math::Q newPose = interpolator.x(time_);
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
    std::cout << "HERE" << std::endl;
    data.close();
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "pathBlend.rwplay");

    std::cout << "Exiting...\n" << std::endl;

    return 0;
}