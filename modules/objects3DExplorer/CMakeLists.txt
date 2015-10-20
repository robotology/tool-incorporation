# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME objects3DExplorer)
PROJECT(${PROJECTNAME})

find_package(ICUB REQUIRED)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)


source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${ICUBCONTRIB_INSTALL_PREFIX}/include)
include_directories(${YarpCloud_INCLUDE_DIRS})
include_directories(${generated_libs_dir}/include)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
link_directories(${ICUBCONTRIB_INSTALL_PREFIX}/lib/x86_64-linux-gnu) #This should be added by default, but it's not, so add it just in case.

add_executable(${PROJECTNAME} ${source} ${header})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES} YarpCloud icubmod )
install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

