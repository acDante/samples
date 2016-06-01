cmake_minimum_required(VERSION 2.8.3)
project(sig_turtlebot_interaction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################





###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES user
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories (/home/sigverse/sigverse-2.3.5/include)
include_directories(/home/sigverse/sigverse-2.3.5/include/sigverse/comm)
SET(CMAKE_CXX_FLAGS "-DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE")


## Build talker and listener
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
find_package(catkin REQUIRED)


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



include_directories(include ${catkin_INCLUDE_DIRS})
ADD_LIBRARY(turtle_interaction src/turtle_interaction.cpp)
target_link_libraries(turtle_interaction ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtle_interaction sig_turtle_interaction_gencpp)


