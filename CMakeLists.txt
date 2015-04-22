# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)
project(toolModeler)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(PCL 1.7 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

add_definitions(${YARP_DEFINES})   # add yarp definitions - this contains also -D_REENTRANT

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()


# setting options
option (BUILD_TOOL_EXPLORER "Build tool Explorer" ON)
option (BUILD_TOOL_3DSHOW "Build 3D show" ON)
option (BUILD_TOOL_FEATEXT "Build tool feature extractor" ON)

# build the library 
add_subdirectory(libYarpCloud)

## add modules
if(BUILD_TOOL_EXPLORER)
    add_subdirectory(modules/toolExplorer)
endif()

if(BUILD_TOOL_3DSHOW)
    add_subdirectory(modules/tool3Dshow)
endif()

if(BUILD_TOOL_FEATEXT)
    add_subdirectory(modules/toolFeatExt)
endif()

## then apps
add_subdirectory(app)

## Add this instruction IF AND ONLY IF the project uses libraries.
icubcontrib_finalize_export (toolModeler)  
## Uninstall
icubcontrib_add_uninstall_target()


