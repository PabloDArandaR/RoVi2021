#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include <rw/math.hpp>
#include <rw/invkin.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/trajectory.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PathPlanner.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rw::pathplanning;
using namespace rwlibs::proximitystrategies;

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