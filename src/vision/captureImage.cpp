#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <stdlib.h> 

#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "aux/stereoAid.cpp"

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// using pclPoint = pcl::PointXYZRGB;
// using pclCloud = pcl::PointCloud<pclPoint>;

typedef rw::models::WorkCell Cell;
typedef rw::kinematics::Frame Frame;
typedef rw::models::Device Device;
typedef rw::kinematics::State State;
typedef rw::sensor::Camera Camera;


using namespace rw::core;
using namespace rw::common;
using rw::graphics::SceneViewer;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;
using namespace rwlibs::simulation;
using namespace rws;

const int width {640}, height {480};
const float fov {50.0};


int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize
    const std::string wcFile = "../resources/Project_WorkCell/Scene.wc.xml";
    std::cout << "Trying to use workcell " << wcFile <<  std::endl;
    std::string deviceName = "UR-6-85-5-A";
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load (wcFile);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set some variables
    rwlibs::simulation::SimulatedCamera::Ptr leftCamera, rightCamera;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set State and Workspace, and initial states and frames
    
    Frame *leftCamera_frame = wc->findFrame("Camera_Left");
    Frame *rightCamera_frame = wc->findFrame("Camera_Right");


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    RobWorkStudioApp app ("");
    RWS_START(app)
    {
        rws::RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->postOpenWorkCell (wcFile);
        rw::common::TimerUtil::sleepMs (500);

        const rw::graphics::SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
        const rwlibs::simulation::GLFrameGrabber::Ptr framegrabber =
            rw::core::ownedPtr (new rwlibs::simulation::GLFrameGrabber (width, height, fov));
        framegrabber->init (gldrawer);
        static const double DT {0.001};
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState();
        int cnt = 0;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Camera left
        rwlibs::simulation::SimulatedCamera::Ptr simcamLeft =
            rw::core::ownedPtr (new rwlibs::simulation::SimulatedCamera ("Camera", fov, leftCamera_frame, framegrabber));
        
        // Initializing camera
        simcamLeft->setFrameRate (100);
        simcamLeft->initialize ();
        simcamLeft->start ();
        simcamLeft->acquire ();

        const Image *imgL;
        cnt = 0;
        std::cout << "[NOTE] Finding left camera.\n";
        while (!simcamLeft->isImageReady())
        {
            if (!simcamLeft->isImageReady()){
                std::cout << "       -Left camera isn't ready\n";
            }
            simcamLeft->update(info, state);
            cnt++;
        }

        std::cout << "[NOTE] Found the cameras and writing image after " << cnt << " tries." << std::endl;
        imgL = simcamLeft->getImage();
        imgL->saveAsPPM("../resources/leftImage.ppm");
        simcamLeft->stop();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Camera right
        rwlibs::simulation::SimulatedCamera::Ptr simcamRight =
            rw::core::ownedPtr (new rwlibs::simulation::SimulatedCamera ("Camera", fov, rightCamera_frame, framegrabber));
        
        // Initializing camera
        simcamRight->setFrameRate (100);
        simcamRight->initialize ();
        simcamRight->start ();
        simcamRight->acquire ();

        const Image *imgR;
        cnt = 0;
        std::cout << "[NOTE] Finding right camera.\n";
        while (!simcamRight->isImageReady())
        {
            if (!simcamRight->isImageReady()){
                std::cout << "       -Right camera isn't ready\n";
            }
            simcamRight->update(info, state);
            cnt++;
        }

        std::cout << "[NOTE] Found the right camera and writing image after " << cnt << " tries." << std::endl;
        imgR = simcamRight->getImage();
        imgR->saveAsPPM("../resources/rightImage.ppm");
        simcamRight->stop();


        app.close();
    };

    RWS_END ();

    return 0;
}

