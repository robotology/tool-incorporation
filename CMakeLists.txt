# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME objects3DModeler)
project(${PROJECTNAME})

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
option (BUILD_OBJECTS3DEXPLORER "Build objects 3D Explorer" ON)
option (BUILD_SHOW3D "Build 3D show module" ON)
option (BUILD_TOOLFEATEXT "Build tool feature extractor" ON)
option (BUILD_OBJFUS3D "Build object Fusion 3D" ON)

# build the library 
add_subdirectory(libYarpCloud)
# add_subdirectory(libcpu_tsdf)

## add modules
if(BUILD_OBJECTS3DEXPLORER)
    add_subdirectory(modules/objects3DExplorer)

endif()

if(BUILD_SHOW3D)
    add_subdirectory(modules/show3D)
endif()

if(BUILD_TOOLFEATEXT)
    add_subdirectory(modules/toolFeatExt)
endif()

if(BUILD_OBJFUS3D)
    add_subdirectory(modules/objFus3D)
endif()

## obsolete
## add_subdirectory(modules/toolRecognizer)

## then apps
add_subdirectory(app)

## Add this instruction IF AND ONLY IF the project uses libraries.
icubcontrib_finalize_export (objects3DModeler)  
## Uninstall
icubcontrib_add_uninstall_target()


