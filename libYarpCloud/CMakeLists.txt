
set(PROJECTNAME YarpCloud)
project(${PROJECTNAME})

SET(YARPCLOUD_HDRS 
    include/iCub/YarpCloud/CloudUtils.h
)

SET(YARPCLOUD_HDRS_IMPL 
)


SET(YARPCLOUD_SRCS 
    src/CloudUtils.cpp
)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUBCONTRIB_INSTALL_PREFIX}/include)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


link_directories(${ICUBCONTRIB_INSTALL_PREFIX}/lib) #This should be added by default, but it's not, so add it just in case.

#include_directories(${toolInc_INCLUDE_DIRS})
#include_directories(${minimumBoundingBox_INCLUDE_DIRS})

include_directories(include ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${YARPCLOUD_HDRS} ${YARPCLOUD_SRCS})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES})
#set_target_properties(${PROJECTNAME} PROPERTIES LINKER_LANGUAGE CXX)

icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
										  EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS}
										  DESTINATION include/iCub/YarpCloud
										  FILES ${YARPCLOUD_HDRS})

