# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME toolExplorer)
PROJECT(${PROJECTNAME})

find_package(PCL 1.7 REQUIRED)
find_package(grasp)

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS}) 
#include_directories(${objects3D_INCLUDE_DIRS})
include_directories(${grasp_INCLUDE_DIRS})  

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(source toolExplorer.cpp)

add_executable(${PROJECTNAME} ${source})
#target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES} objects3D icubmod ctrlLib)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES} grasp icubmod ctrlLib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

#icubcontrib_finalize_export (${PROJECTNAME})

