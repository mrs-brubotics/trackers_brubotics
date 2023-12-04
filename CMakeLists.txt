cmake_minimum_required(VERSION 3.5)
project(trackers_brubotics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  geometry_msgs
  tf
  std_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_managers
  dynamic_reconfigure
  # additionally added by brubotics:
  mrs_uav_trackers
  message_generation
  genmsg
  controllers_brubotics
  eigen_conversions #  additionally added by Titouan and Jonathan:
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

add_message_files(FILES
  DSM.msg
  FutureTrajectoryTube.msg
  DistanceBetweenUavs.msg
  TrajectoryTracking.msg
  ComputationalTime.msg
  )
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

generate_dynamic_reconfigure_options(
  cfg/mpc_copy_tracker.cfg
  cfg/bypass_tracker.cfg
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

set(LIBRARIES
  TrackersBrubotics_MpcCopyTracker
  # TODO ADD
  TrackersBrubotics_DergbryanTracker
  TrackersBrubotics_BypassTracker
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}   
  # additionally added by brubotics wrt ctu:
  message_runtime
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

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
  MESSAGE(FATAL_ERROR "Mising MpcCopyTrackerSolver.so for armhf")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "i386")
  MESSAGE(FATAL_ERROR "Mising MpcCopyTrackerSolver.so for i386")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "x64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcCopyTrackerSolver/x64/libMpcCopyTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "amd64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcCopyTrackerSolver/x64/libMpcCopyTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "arm64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcCopyTrackerSolver/arm64/libMpcCopyTrackerSolver.so)
else()
  MESSAGE(FATAL_ERROR ${CMAKE_DEB_HOST_ARCH})
  MESSAGE(FATAL_ERROR "MpcTrackerSolver.so has not been selected, check CMakeLists.txt.")
endif()

# MPCCopyTracker

add_library(TrackersBrubotics_MpcCopyTracker
  src/mpc_copy_tracker/mpc_copy_tracker.cpp
  )

add_dependencies(TrackersBrubotics_MpcCopyTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(TrackersBrubotics_MpcCopyTracker
  ${catkin_LIBRARIES}
  ${MPC_CONTROLLER_SOLVER_BIN}
  )


# BypassTracker

add_library(TrackersBrubotics_BypassTracker
  src/bypass_tracker/bypass_tracker.cpp
  )

add_dependencies(TrackersBrubotics_BypassTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(TrackersBrubotics_BypassTracker
  ${catkin_LIBRARIES}
  )

# DergbryanTracker

add_library(TrackersBrubotics_DergbryanTracker
src/dergbryan_tracker/dergbryan_tracker.cpp
)

add_dependencies(TrackersBrubotics_DergbryanTracker
${catkin_EXPORTED_TARGETS}
${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(TrackersBrubotics_DergbryanTracker
${catkin_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES ${MPC_CONTROLLER_SOLVER_BIN}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )