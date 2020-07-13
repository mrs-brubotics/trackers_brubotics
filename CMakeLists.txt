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
  mrs_uav_trackers
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
    cfg/mpc_tracker_interns.cfg
    )

include_directories(${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -O0 -g")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MpcTrackerCvxgen
  CATKIN_DEPENDS geometry_msgs tf mrs_lib mrs_uav_managers mrs_msgs mrs_uav_trackers
  DEPENDS Eigen
  )

###########
## Build ##
###########

# Line tracker

add_library(LineTrackerInterns
  src/line_tracker_interns/line_tracker_interns.cpp
  )
add_dependencies(LineTrackerInterns
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
target_link_libraries(LineTrackerInterns
  ${catkin_LIBRARIES}
  )

  # MPC tracker

  add_library(MpcTrackerInterns
    src/mpc_tracker_interns/mpc_tracker_interns.cpp
    )
  add_dependencies(MpcTrackerInterns
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(MpcTrackerInterns
    ${catkin_LIBRARIES}
    )

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


  # AlphaDrg

  add_library(AlphaDrg
    src/alpha_drg/alpha_drg.cpp
    )
  add_dependencies(AlphaDrg
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(AlphaDrg
    ${catkin_LIBRARIES}
    )
    
#############
## Install ##
#############

install(TARGETS
  LineTrackerInterns
  MpcTrackerInterns
  BypassTracker
AlphaDrg
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
