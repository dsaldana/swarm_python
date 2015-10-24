cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#################################################
# Find the Swarm library.
find_package(swarm QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SWARM_CXX_FLAGS}")
include_directories(${SWARM_INCLUDE_DIRS})
link_directories(${SWARM_LIBRARY_DIRS})

#################################################
# Generate the team's plugin for controlling a robot.
add_library(TeamControllerPlugin SHARED TeamControllerPlugin.cc)
target_link_libraries(TeamControllerPlugin ${SWARM_LIBRARIES})
# GNUInstallDirst defines CMAKE_INSTALL_LIBDIR, among other things.
include(GNUInstallDirs)
install (TARGETS TeamControllerPlugin DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib)



#### Python libraries #########
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
#add_executable(TeamControllerPlugin ${SOURCE_FILES})
target_link_libraries(TeamControllerPlugin ${PYTHON_LIBRARIES})
