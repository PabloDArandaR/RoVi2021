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

#include "stereoAid.cpp"

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

const int width {200}, height {200};
const float fov {1.0};


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
        rw::common::TimerUtil::sleepMs (5000);

        const rw::graphics::SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
        const rwlibs::simulation::GLFrameGrabber::Ptr framegrabber =
            rw::core::ownedPtr (new rwlibs::simulation::GLFrameGrabber (width, height, fov));
        framegrabber->init (gldrawer);
        rwlibs::simulation::SimulatedCamera::Ptr simcamLeft =
            rw::core::ownedPtr (new rwlibs::simulation::SimulatedCamera ("SimulatedCamera", fov, leftCamera_frame, framegrabber));
        rwlibs::simulation::SimulatedCamera::Ptr simcamRight =
            rw::core::ownedPtr (new rwlibs::simulation::SimulatedCamera ("SimulatedCamera", fov, rightCamera_frame, framegrabber));
        
        // Initializing cameras
        simcamLeft->setFrameRate (100);
        simcamLeft->initialize ();
        simcamLeft->start ();
        simcamLeft->acquire ();
        simcamRight->setFrameRate (100);
        simcamRight->initialize ();
        simcamRight->start ();
        simcamRight->acquire ();

        static const double DT {0.001};
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState();
        int cnt = 0;
        const Image *imgL, *imgR;
        while ((!simcamLeft->isImageReady()) && (!simcamRight->isImageReady()))
        {
            std::cout << "[NOTE] At least one of the cameras is not ready." << std::endl;
            simcamLeft->update(info, state);
            simcamRight->update(info, state);
            cnt++;
        }

        imgL = simcamLeft->getImage();
        imgR = simcamRight->getImage();

        imgL->saveAsPPM("leftImage.ppm");
        imgR->saveAsPPM("rightImage.ppm");

        simcamLeft->stop();
        simcamRight->stop();
        app.close();
    };

    RWS_END ();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Obtain the images

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Dense stereo implementation 

    // Rectify and undistort images (if necessary)
    
        //Since the cameras are parallel, the images are rectified already

    // Compute disparity map
    //cv::Mat colors = left;
    //cv::cvtColor(right, right, cv::COLOR_RGB2GRAY);
    //cv::cvtColor(left, left, cv::COLOR_RGB2GRAY);

    //cv::Mat disp = DisparitySGBM(left, right, 21, 10);

    // Compute point cloud
    //cv::Mat points = reproject3D(disp, defineQ(width, height));
    //pclCloud::Ptr pointCloud = obtainPointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z)

    // Filter/segment the point cloud

    // Global pose estimation to find the object pose

    // Local pose estimation to find the object pose

    return 0;
}

