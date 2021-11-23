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
#include <rw/sensor/Camera.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
//#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

int main()
{

    // Declare main variables

    // Obtain the pictures from the cameras

    // Find features

    // Match features from one image in the other

    // Find the transformation of one image to the other

    // Should we do the following ones?

        // Compute the disparity map

        // Compute the point cloud

    // Compute the pose of the object

    return 0;
}