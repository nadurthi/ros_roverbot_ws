cmake_minimum_required(VERSION 2.8.3)
project(roverbot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra -pedantic)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -pedantic -std=c++1y ${CMAKE_CXX_FLAGS}")
endif()

set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_TOOLKIT_ROOT_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rosutils
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  image_transport
  
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package (Threads)
find_package (Eigen3 )
# find_package(realsense2)
find_package(OpenCV REQUIRED )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/Roverbot.cfg
  #...
)



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roverbot
#  CATKIN_DEPENDS roscpp rospy std_msgs
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime cv_bridge image_transport 
#  DEPENDS system_lib
)

#-------------------------------------------------------
# header and cpputility_SOURCES
#-------------------------------------------------------
set(HEADER_BASE "${CMAKE_CURRENT_SOURCE_DIR}/include")
# message("----------------------------------------------------------${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB EXT_SOURCES_BASE "extsrc/*.cpp")

###########
## Build ##
###########

# add_executable(roverimupub src/rover_publish_imu.cpp)
# target_sources(roverimupub PUBLIC ${EXT_SOURCES_BASE})
# target_link_libraries(roverimupub ${catkin_LIBRARIES}  ${OpenCV_LIBS}  Eigen3::Eigen)
# target_include_directories(roverimupub PUBLIC ${HEADER_BASE})
# add_dependencies(roverimupub roverbot_generate_messages_cpp )
# add_dependencies(roverimupub ${PROJECT_NAME}_gencfg )


# add_executable(roverpwmctrl src/rover_pwm_ctrl.cpp)
# target_sources(roverpwmctrl PUBLIC ${EXT_SOURCES_BASE})
# target_link_libraries(roverpwmctrl ${catkin_LIBRARIES}  ${OpenCV_LIBS}  Eigen3::Eigen)
# target_include_directories(roverpwmctrl PUBLIC ${HEADER_BASE})
# add_dependencies(roverpwmctrl roverbot_generate_messages_cpp)
# add_dependencies(roverpwmctrl ${PROJECT_NAME}_gencfg )


# add_executable(roversonarpub src/rover_publish_sonar.cpp)
# target_sources(roversonarpub PUBLIC ${EXT_SOURCES_BASE})
# target_link_libraries(roversonarpub ${catkin_LIBRARIES}  ${OpenCV_LIBS}  Eigen3::Eigen)
# target_include_directories(roversonarpub PUBLIC ${HEADER_BASE})
# add_dependencies(roversonarpub roverbot_generate_messages_cpp)
# add_dependencies(roversonarpub ${PROJECT_NAME}_gencfg )


# add_executable(roverbatterypub src/rover_battery_publish.cpp)
# target_sources(roverbatterypub PUBLIC ${EXT_SOURCES_BASE})
# target_link_libraries(roverbatterypub ${catkin_LIBRARIES}  ${OpenCV_LIBS}  Eigen3::Eigen)
# target_include_directories(roverbatterypub PUBLIC ${HEADER_BASE})
# add_dependencies(roverbatterypub roverbot_generate_messages_cpp)
# add_dependencies(roverbatterypub ${PROJECT_NAME}_gencfg )

add_executable(rovercampub src/rover_cam_publish.cpp)
target_sources(rovercampub PUBLIC ${EXT_SOURCES_BASE})
target_link_libraries(rovercampub ${catkin_LIBRARIES}  ${OpenCV_LIBS}  Eigen3::Eigen)
target_include_directories(rovercampub PUBLIC ${HEADER_BASE})
add_dependencies(rovercampub roverbot_generate_messages_cpp)
add_dependencies(rovercampub ${PROJECT_NAME}_gencfg )


add_executable(roveri2clooper src/rover_i2c_looper.cpp)
target_sources(roveri2clooper PUBLIC ${EXT_SOURCES_BASE})
target_link_libraries(roveri2clooper ${catkin_LIBRARIES}  ${OpenCV_LIBS}  Eigen3::Eigen)
target_include_directories(roveri2clooper PUBLIC ${HEADER_BASE})
add_dependencies(roveri2clooper roverbot_generate_messages_cpp)
add_dependencies(roveri2clooper ${PROJECT_NAME}_gencfg )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/roverbot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/roverbot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roverbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
