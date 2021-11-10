#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include <rw/math.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/trajectory.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <fstream>

rw::trajectory::LinearInterpolator<rw::math::Q> linearInterpolator(rw::math::Q initial, rw::math::Q final, double duration)
{
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(initial, final, duration);

    return interpolator;
}

std::vector<rw::math::Q> linearPathFromInterpolator(rw::trajectory::LinearInterpolator<rw::math::Q> interpolator, double precision)
{
    // Basic variables
    std::vector<rw::math::Q> path;

    double duration = interpolator.duration();
    int n_steps = duration/precision;

    // Path obtention
    for (int i = 0; i < n_steps; i++)
    {
        path.push_back(interpolator.x(precision*i));
    }

    return path;
}

std::vector<rw::math::Q> linearInterpolation(rw::math::Q initial, rw::math::Q final, double duration, double precision)
{
    // Basic variables
    rw::trajectory::LinearInterpolator<rw::math::Q> interpolator(initial, final, duration);
    std::vector<rw::math::Q> path;
    int n_steps = duration/precision;

    // Path obtention
    for (int i = 0; i <= n_steps; i++)
    {
        path.push_back(interpolator.x(precision*i));
    }

    return path;
}

void savePath(std::vector<rw::math::Q> path, std::vector<double> time, std::string scene, std::string filesave, std::string device_name)
{

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(scene);
    if (wc == NULL)
    {
        std::cerr << "[WorkCell] scene in location " << scene << " not loaded." << std::endl;
    }

	rw::kinematics::State state = wc->getDefaultState();
	rw::models::SerialDevice::Ptr device = wc->findDevice<rw::models::SerialDevice>(device_name);
    if (device == NULL)
    {
        std::cerr << "[Device] Device with name " <<  device_name << "not found." << std::endl;
    }
    rw::trajectory::TimedStatePath pathStates;

    if (path.size() != time.size())
    {
        std::cerr << "[savePath] Vectors of different size. (" << path.size() << " and " << time.size() <<")\n";
    }

    for (int i = 0; i < (int)path.size(); i++)
    {
		device->setQ(path[i], state);
        pathStates.push_back(rw::trajectory::TimedState(time[i],state));
    }

	rw::loaders::PathLoader::storeTimedStatePath(*wc, pathStates, filesave);
}

template <class T>
std::vector<T> timeVector(T duration, T precision)
{
    std::vector<T> time;
    int n_steps {(int)(duration/precision)};
    for (int i = 0; i<=n_steps; i++)
    {
        time.push_back(precision*i);
    }

    return time;
}

/* 
//TODO
std::vector<rw::math::Q> initialFinalQ(std::string filename)
{
    std::vector<double> pose;
    int n_lines {0}, n_initial {0}, n_final {0};
    rw::math::Q initial, final;
    std::string line;
    std::ifstream file(filename);

    // Check format of the file
    while (std::getline(file, line))
    {
        n_lines++;
    }
    if (n_lines != 2)
    {
        std::cerr << "[File reading] Number of lines in the initial and final pose files is incorrect.\n" std::endl;
    }

} */

std::vector<rw::math::Q> parabolicBlend(rw::math::Q initialPose, rw::math::Q endPose, double totalTime, rw::math::Q linearVelocity, rw::math::Q blendAcceleration, double blendTime, double precision)
{
    std::vector<rw::math::Q> result;
    int blendSteps {(int)(blendTime/precision)}, linearSteps {(int)((totalTime - blendTime)/precision)};

    // Starting blend
    for (int i = 0; i < blendSteps; i++)
    {
        result.push_back(initialPose + (1/2)*blendAcceleration*(precision*i));
    }
    // Linear part
    for (int i = blendSteps; i < linearSteps + blendSteps; i++)
    {
        result.push_back(initialPose + blendAcceleration*blendTime*(i*precision - blendTime/2));
    }

    // End blend
    for (int i = linearSteps + blendSteps; i < linearSteps + 2*blendSteps; i++)
    {
        result.push_back(endPose - (1/2)*blendAcceleration*(totalTime -precision*i));
    }

    return result;
}

void storePosePath(std::string fileName, std::vector<rw::math::Q> path, std::vector<double> timeVector, int nPoints, int nJoints)
{
    std::fstream file;
    file.open(fileName);

    // Generate the tags
    for (int i = 0; i < nJoints; i++)
    {
        file << "joint" << i << ",";
    }
    file << "time\n";

    // Store the path
    for (int i = 0; i < nPoints; i++)
    {
        for (int j = 0; j < nJoints; j++)
        {
            file << path[i][j] << ",";
        }
        file << timeVector[i] << std::endl;
    }
    file.close();
}