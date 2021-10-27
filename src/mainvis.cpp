#include <iostream>
#include <string>
#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
//#include <SamplePlugin.hpp>

using rw::loaders::WorkCellLoader;

int main()
{
    /////////////////////////////////////////////////////////////////////////////////////////
    // Setup of WorkCell, plug-ins, and any other required piece to evaluate.
    const std::string scenePath = "../resources/Project_WorkCell/Scene.wc.xml";
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::WorkCellLoader::Factory::load(scenePath);

    if (wc.isNull())
    {
        std::cout << "[SETUP]Error while loading the WorkCell" << std::endl;
        return 1;
    }

    // Check the plugins interface

    return 0;
}
