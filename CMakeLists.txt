cmake_minimum_required(VERSION 3.1.0)

project(RoVi2021)
# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(ROBOT ${ROOT}/src/mainrob.cpp)
SET(VISION ${ROOT}/src/mainvis.cpp)

# Setting up compiler version
add_compile_options(-std=c++11)

# Add the folder with include files
include_directories(${ROOT}/include)
include_directories(${Qt5Widgets_INCLUDE_DIRS})

# Set the RobWork root
SET(RW_ROOT $ENV{RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
#[[set(OPENCV_ROOT $ENV{OPENCV_ROOT})
set(OpenCV_DIR ${OPENCV_ROOT}/cmake)]]
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
SET(RWSIMULATION_ROOT $ENV{RWSIM_ROOT})
SET(RobWorkSim_DIR ${RWSIMULATION_ROOT}/cmake)

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Various setups
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${OPENCV_ROOT}/build ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${RWSTUDIO_ROOT}/build ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${RWSIM_ROOT}/build ${CMAKE_MODULE_PATH})

#QT for the simulations
#find_package(Qt5 COMPONENTS Widgets REQUIRED)
#include_directories(${Qt5Widgets_INCLUDE_DIRS})

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})

# Use RobWork
find_package(RobWork REQUIRED)
include_directories( ${ROBWORK_INCLUDE_DIRS} )
link_directories( ${ROBWORK_LIBRARY_DIRS} )
add_definitions(${ROBWORK_DEFINITIONS})

# Use RobWorkStudio
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

# Use RobWorkStudio
FIND_PACKAGE(RobWorkSim REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS})

# Use PCL
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_SEARC_INCLUDE_DIRS})
link_directories(${PCL_SEARCH_LIBRARY_DIRS})
add_definitions(${PCL_SEARCH_DEFINITIONS})


# plugin
include_directories(PRIVATE resources/RoviSamplePlugin/src)

add_executable(rrt src/rob/RRT.cpp)
add_executable(pointToPoint src/rob/pointToPoint.cpp)
add_executable(parabolicBlend src/rob/parabolicBlend.cpp)
add_executable(denseStereo src/vision/denseStereo.cpp)

target_link_libraries(rrt ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(pointToPoint ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(parabolicBlend ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(denseStereo ${ROBWORK_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS})
