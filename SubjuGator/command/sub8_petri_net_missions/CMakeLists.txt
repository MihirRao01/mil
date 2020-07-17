cmake_minimum_required(VERSION 3.0.2)
project(sub8_petri_net_missions)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mil_poi mil_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
  INCLUDE_DIRS include
#  LIBRARIES sub8_missions2
  CATKIN_DEPENDS roscpp rospy std_msgs mil_poi mil_msgs geometry_msgs
  DEPENDS system_lib pthread
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(PETRI_NET_DIR ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/petri_nets)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PETRI_NET_DIR}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sub8_missions2.cpp
# )


# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide

add_custom_target(${PROJECT_NAME}_petri_net_dir
COMMAND mkdir -p ${PETRI_NET_DIR})

set(${PROJECT_NAME}_start_gate_net_cpp
    ${PETRI_NET_DIR}/start_gate_net.cpp)

set(${PROJECT_NAME}_start_gate_state_graph_dot
    ${PETRI_NET_DIR}/start_gate_state_graph.dot)

add_executable(${PROJECT_NAME}_start_gate
               src/start_gate/start_gate.cpp
               ${PROJECT_NAME}_start_gate_net_cpp)
add_dependencies(${PROJECT_NAME}_start_gate
               ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(${PROJECT_NAME}_start_gate
               PROPERTIES COMPILE_FLAGS "-std=c++14 -pthread -Wall")

# source fiels that infomr the petri net
get_filename_component(START_GATE_NET_DOT
                       "${CMAKE_CURRENT_SOURCE_DIR}/src/start_gate/start_gate_net.dot"
                       ABSOLUTE)
get_filename_component(START_GATE_HPP
                       "${CMAKE_CURRENT_SOURCE_DIR}/include/sub8_missions2/start_gate.hpp"
                       ABSOLUTE)
# auto generated files for the petri net interpreter to write
get_filename_component(START_GATE_NET_CPP
                       "${PETRI_NET_DIR}/start_gate_net.cpp"
                       ABSOLUTE)
get_filename_component(START_GATE_NET_HPP
                       "${PETRI_NET_DIR}/start_gate_net.hpp"
                       ABSOLUTE)

add_custom_command(OUTPUT ${PROJECT_NAME}_start_gate_net_cpp
                          ${PROJECT_NAME}_start_gate_net_hpp
                          ${PROJECT_NAME}_start_gate_state_graph_dot
                   DEPENDS src/start_gate/start_gate_net.dot ${PROJECT_NAME}_petri_net_dir
COMMAND   make_petri_net -dot ${START_GATE_NET_DOT} -header ${START_GATE_NET_HPP} -source ${START_GATE_NET_CPP} -types ${START_GATE_HPP} -os linux_cpp --debug print --state_graph_type dot --state_graph_output ${PETRI_NET_DIR}/start_gate_state_graph.dot)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_start_gate
  ${catkin_LIBRARIES} ${boost_LIBRARIES}
)

unset(PETRI_NET_DIR)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sub8_missions2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
