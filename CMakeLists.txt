cmake_minimum_required(VERSION 3.1.2)
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
  dynamic_reconfigure
  # additionally added by brubotics wrt ctu:
  mrs_uav_trackers 
  message_generation
  genmsg
  )

add_message_files(FILES
  DSM.msg
  FutureTrajectoryTube.msg
  )
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

generate_dynamic_reconfigure_options(
    cfg/mpc_copy_tracker.cfg
    ) 


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES MpcTrackerCvxgen # additionally added by brubotics wrt ctu
  CATKIN_DEPENDS geometry_msgs tf mrs_lib mrs_uav_managers mrs_msgs 
  # additionally added by brubotics wrt ctu:
  mrs_uav_trackers message_runtime std_msgs 
  DEPENDS Eigen
  )

###########
## Build ##
###########

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

# MpcCopyTracker

add_library(MpcCopyTracker src/mpc_copy_tracker/mpc_copy_tracker.cpp)

add_dependencies(MpcCopyTracker
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)
target_link_libraries(MpcCopyTracker
	${catkin_LIBRARIES}
	${MPC_CONTROLLER_SOLVER_BIN}
)

# BypassTracker

add_library(BypassTracker 
src/bypass_tracker/bypass_tracker.cpp)
add_dependencies(BypassTracker
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)
target_link_libraries(BypassTracker
	${catkin_LIBRARIES}
)

# DergbryanTracker

add_library(DergbryanTracker
src/dergbryan_tracker/dergbryan_tracker.cpp
)
add_dependencies(DergbryanTracker
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	#${PROJECT_NAME}_generate_messages_cpp}
)
target_link_libraries(DergbryanTracker
	${catkin_LIBRARIES}
)
  
#############
## Install ##
#############

install(TARGETS  
  MpcCopyTracker
  BypassTracker
  DergbryanTracker
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
