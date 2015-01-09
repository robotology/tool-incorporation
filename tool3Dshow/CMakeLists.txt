CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME tool3Dshow)
PROJECT(${PROJECTNAME})

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${YARP_INCLUDE_DIRS})

set(source tool3Dshow.cpp)

add_executable(${PROJECTNAME} ${source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

