# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME toolExplorer)
PROJECT(${PROJECTNAME})

find_package(ICUB REQUIRED)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)


source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${ICUBCONTRIB_INSTALL_PREFIX}/include)
include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${objects3D_INCLUDE_DIRS})
include_directories(${minimumBoundingBox_INCLUDE_DIRS})
include_directories(${YarpCloud_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${ICUBCONTRIB_INSTALL_PREFIX}/lib) #This should be added by default, but it's not, so add it just in case.
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} objects3D minimumBoundingBox YarpCloud icubmod ${PCL_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

