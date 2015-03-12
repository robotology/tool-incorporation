# Copyright: (C) 2014 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME tool3Dshow)
PROJECT(${PROJECTNAME})

find_package(PCL 1.7 REQUIRED)

#include_directories(${ICUB_INCLUDE_DIRS} 
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUBCONTRIB_INSTALL_PREFIX}/include)
include_directories(${objects3D_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${ICUBCONTRIB_INSTALL_PREFIX}/lib) #This should be added by default, but it's not, so add it just in case.
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(source tool3Dshow.cpp)

add_executable(${PROJECTNAME} ${source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES} objects3D )
install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
