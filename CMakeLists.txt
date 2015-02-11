# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6.9)
project(toolModeler)

find_package(YARP)
find_package(ICUB)
find_package(ICUBcontrib REQUIRED)
#find_package(PCL 1.7)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES})   # add yarp definitions - this contains also -D_REENTRANT

## add modules

message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
add_subdirectory(toolExplorer)
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
message("----------------------------------------------------")
add_subdirectory(mergeClouds)
add_subdirectory(tool3Dshow)
add_subdirectory(toolFeatExt)

## then apps
add_subdirectory(app)

## Add this instruction IF AND ONLY IF the project uses libraries.
#icubcontrib_finalize_export (toolModeler)  

## Uninstall
icubcontrib_add_uninstall_target()

