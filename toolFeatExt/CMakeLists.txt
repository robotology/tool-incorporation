# Copyright: (C) 2014 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME toolFeatExt)
PROJECT(${PROJECTNAME})

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${YARP_INCLUDE_DIRS})

set(source toolFeatExt.cpp)

add_executable(${PROJECTNAME} ${source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
