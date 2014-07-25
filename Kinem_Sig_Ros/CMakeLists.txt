cmake_minimum_required(VERSION 2.8.3)
project(Kinem_Sig_Ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Robot_Joint.msg
   Init_Pos.msg
)

#add_message_files(DIRECTORY msg FILES Robot_Joint.msg)
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   AddTwoInts.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${roscpp_INCLUDE_DIRS})

#############
## Testing ##
#############

## Build talker and listener
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED)

find_package(orocos_kdl)
find_package(kdl_parser)



IF (NOT ODE_FOUND)
	
	  FIND_PATH(ODE_INCLUDE_DIR ode/ode.h
	    /usr/include
	    /usr/local/include
	    $ENV{OGRE_HOME}/include # OGRE SDK on WIN32
	    $ENV{INCLUDE}
	  )
	  FIND_LIBRARY(ODE_LIBRARY
	    NAMES ode
	    PATHS
	    /usr/lib
	    /usr/local/lib
	    $ENV{OGRE_HOME}/lib # OGRE SDK on WIN32
	  )
	
	  IF(ODE_INCLUDE_DIR)
	    MESSAGE(STATUS "Found ODE include dir: ${ODE_INCLUDE_DIR}")
	  ELSE(ODE_INCLUDE_DIR)
	    MESSAGE(STATUS "Could NOT find ODE headers.")
	  ENDIF(ODE_INCLUDE_DIR)
	
	  IF(ODE_LIBRARY)
	    MESSAGE(STATUS "Found ODE library: ${ODE_LIBRARY}")
	  ELSE(ODE_LIBRARY)
	    MESSAGE(STATUS "Could NOT find ODE library.")
	  ENDIF(ODE_LIBRARY)
	
	  IF(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	     SET(ODE_FOUND TRUE CACHE STRING "Whether ODE was found or not")
	   ELSE(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	     SET(ODE_FOUND FALSE)
	     IF(ODE_FIND_REQUIRED)
	       MESSAGE(FATAL_ERROR "Could not find ODE. Please install ODE (http://www.ode.org)")
	     ENDIF(ODE_FIND_REQUIRED)
	   ENDIF(ODE_INCLUDE_DIR AND ODE_LIBRARY)
	ENDIF (NOT ODE_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS}  ${orocos_kdl_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(/home/eric/sigverse-2.2.2/include/sigverse) 
include_directories(/home/eric/sigverse-2.2.2/include/sigverse/comm/controller)
SET(CMAKE_CXX_FLAGS "-DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE")

add_executable(Arm_Control1 src/Arm_Control1.cpp)
#ADD_LIBRARY(Arm_Control1 src/Arm_Control1.cpp)
target_link_libraries(Arm_Control1 ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${Boost_LIBRARIES} ${kdl_parser_LIBRARIES})
add_dependencies(Arm_Control1 Kinem_Sig_Ros_gencpp)

ADD_LIBRARY(Sig_app_kinem src/Sig_app_kinem.cpp)
target_link_libraries(Sig_app_kinem ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES})
add_dependencies(Sig_app_kinem Kinem_Sig_Ros_gencpp)
