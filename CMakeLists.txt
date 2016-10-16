cmake_minimum_required(VERSION 2.8.3)
project(reactor_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## If MoveIt exists, build twist controller
find_package(moveit_core QUIET)
if(${moveit_core_FOUND})
  find_package(catkin REQUIRED
    cmake_modules
    moveit_core
    moveit_msgs
    moveit_ros_planning
  )

  if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  endif()
  set(BUILD_FLAGS "-std=c++0x -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wpedantic -Werror")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

  catkin_package()

  include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
  )

  link_libraries(
    ${catkin_LIBRARIES}
  )

  add_executable(twist_controller src/twist_controller.cpp)
  add_executable(dynamixel_to_joint_state src/dynamixel_to_joint_state.cpp)
else()
  message(WARNING "MoveIt not found, so not building controllers.")
  catkin_package()
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(reactor_controller
#   src/${PROJECT_NAME}/reactor_controller.cpp
# )

## Declare a cpp executable
# add_executable(reactor_controller_node src/reactor_controller_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(reactor_controller_node reactor_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(reactor_controller_node
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
# install(TARGETS reactor_controller reactor_controller_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_reactor_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
