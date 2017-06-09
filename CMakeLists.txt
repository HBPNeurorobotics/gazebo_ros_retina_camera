cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_retina_camera)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
)

###########
## Build ##
###########

SET(RETINA_INCLUDE_DIR $ENV{RETINA_INSTALL_DIR}/include)
SET(CIMG_INCLUDE_DIR $ENV{RETINA_INSTALL_DIR}/CImg-1.6.0_rolling141127)
SET(PYTHON_INCLUDE_DIR /usr/include/python2.7/)
SET(RETINA_LIB_DIR $ENV{RETINA_INSTALL_DIR}/build/lib)

link_directories(${GAZEBO_LIBRARY_DIRS} ${RETINA_LIB_DIR})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIR}
                     ${GAZEBO_INCLUDE_DIRS}
                     ${OGRE_INCLUDE_DIRS}/Paging
                     ${RETINA_INCLUDE_DIR}
                     ${CIMG_INCLUDE_DIR}
                     ${PYTHON_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} src/gazebo_ros_retina_camera.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} CameraPlugin retina)