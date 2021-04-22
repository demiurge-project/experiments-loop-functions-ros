#
# Set minimum required version
#
cmake_minimum_required(VERSION 2.8)
project(experiments_loop_functions)
##ros part

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  LIBRARIES template_controller
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs nav_msgs tf
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

#
# Make sure we are under Unix
#
if(NOT UNIX)
  message(FATAL_ERROR "ARGoS compiles only under UNIX, sorry!")
endif(NOT UNIX)

#
# Set build options
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)

#
# Set compiler flags
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)

#
# Check for libraries
#
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)

#
# Set base source files
#
add_subdirectory(src)
#add_subdirectory(manual-controllers)

if(ARGOS_BUILD_FOR_SIMULATOR)
	add_subdirectory(loop-functions)
endif(ARGOS_BUILD_FOR_SIMULATOR)
