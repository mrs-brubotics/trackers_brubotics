cmake_minimum_required(VERSION 3.0.2)
project(trackers_brubotics)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  tf
  std_msgs
  roscpp
  rospy
  mrs_lib
  mrs_msgs
  mrs_uav_managers
  mrs_uav_trackers # additionally added by brubotics wrt ctu
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
    #cfg/mpc_tracker_copy.cfg
    ) 

#include_directories(${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -O0 -g")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MpcTrackerCvxgen # additionally added by brubotics wrt ctu
  CATKIN_DEPENDS geometry_msgs tf mrs_lib mrs_uav_managers mrs_msgs mrs_uav_trackers # mrs_uav_trackers additionally added by brubotics wrt ctu
  DEPENDS Eigen
  )

###########
## Build ##
###########

#################################### this block (= copy of ctu) was initially not added by interns
include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Mpc Solver Library

# Store in CMAKE_DEB_HOST_ARCH var the current build architecture
execute_process(COMMAND
  dpkg-architecture
  -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
  CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

# deduce the library path based on the system architecture
if(${CMAKE_DEB_HOST_ARCH} MATCHES "armhf")
  MESSAGE(FATAL_ERROR "Mising MpcTrackerSolver.so for armhf")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "i386")
  MESSAGE(FATAL_ERROR "Mising MpcTrackerSolver.so for i386")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "x64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/x64/libMpcTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "amd64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/x64/libMpcTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "arm64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/arm64/libMpcTrackerSolver.so)
else()
  MESSAGE(FATAL_ERROR ${CMAKE_DEB_HOST_ARCH})
  MESSAGE(FATAL_ERROR "MpcTrackerSolver.so has not been selected, check CMakeLists.txt.")
endif()
####################################

  # Bypass tracker

  add_library(BypassTracker
    src/bypass_tracker/bypass_tracker.cpp
    )
  add_dependencies(BypassTracker
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(BypassTracker
    ${catkin_LIBRARIES}
    )


 

  # DergTracker

#  add_library(DergTracker
#    src/derg_tracker/derg_tracker.cpp
#    )
#  add_dependencies(DergTracker
#    ${catkin_EXPORTED_TARGETS}
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    )
#  target_link_libraries(DergTracker
#    ${catkin_LIBRARIES}
#    )
    

  # MpcTrackerCopy

#  add_library(MpcTrackerCopy
#    src/mpc_tracker_copy/mpc_tracker_copy.cpp
#    )
#  add_dependencies(MpcTrackerCopy
#    ${catkin_EXPORTED_TARGETS}
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    )
#  target_link_libraries(MpcTrackerCopy
#    ${catkin_LIBRARIES}
#    )
    
    
#############
## Install ##
#############

install(TARGETS
  BypassTracker
  #DergTracker
  #MpcTrackerCopy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(FILES
  plugins.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
