# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME toolExplorer)
project(${PROJECTNAME})

find_package(PCL 1.7 REQUIRED)

set(source toolExplorer.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(${objects3D_INCLUDE_DIRS}) 
add_executable(${PROJECTNAME} ${source})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES} objects3D icubmod ctrlLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
