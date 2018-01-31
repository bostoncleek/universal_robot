cmake_minimum_required(VERSION 2.8.3)
project(ur_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(AMINO)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_description
#  CATKIN_DEPENDS urdf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

option(COMPILE_SCENEGRAPHS "Compiles scenegraphs from the urs using Amino." OFF)

if (AMINO_FOUND AND COMPILE_SCENEGRAPHS)
    include_directories(include ${AMINO_INCLUDE_DIRS})

    ## Declare a cpp library

    ## Set library and generated file names.
    set(ONE_ARM_C one-arm-scene.c)
    set(ONE_ARM_LIB kavrakilab-one-arm-scene)
    set(TWO_ARM_C two-arm-scene.c)
    set(TWO_ARM_LIB kavrakilab-two-arm-scene)
    set(ENV{ROS_PACKAGE_PATH}
        "${CMAKE_SOURCE_DIR}/..:${CMAKE_SOURCE_DIR}/../../robotiq_85_gripper:$ENV{ROS_PACKAGE_PATH}")

    add_custom_command(OUTPUT ur5_robotiq_robot_limited.urdf COMMAND
                        rosrun xacro xacro --inorder -o ur5_robotiq_robot_limited.urdf ${CMAKE_CURRENT_SOURCE_DIR}/urdf/ur5_robotiq_robot_limited.urdf.xacro)
    # env vars SET in CMakeLists.txt only take effect for cmake itself,
    # so we have to wrap the call with `cmake -E env`
    add_custom_command(OUTPUT ${ONE_ARM_C} COMMAND
                       ${CMAKE_COMMAND} -E env
                       "ROS_PACKAGE_PATH=$ENV{ROS_PACKAGE_PATH}"
                       aarxc ur5_robotiq_robot_limited.urdf -o ${ONE_ARM_C} -n "kavrakilab")
    set_source_files_properties(ur5_robotiq_robot_limited.urdf PROPERTIES GENERATED TRUE)
    set_source_files_properties(${ONE_ARM_C} PROPERTIES GENERATED TRUE)
    add_custom_target(${ONE_ARM_C} DEPENDS ur5_robotiq_robot_limited.urdf)
    add_library(${ONE_ARM_LIB} SHARED ${ONE_ARM_C})

    ## Both arm scene library.
    add_custom_command(OUTPUT ${TWO_ARM_C} COMMAND
                       ${CMAKE_COMMAND} -E env
                       "ROS_PACKAGE_PATH=$ENV{ROS_PACKAGE_PATH}"
                       aarxc ${CMAKE_CURRENT_SOURCE_DIR}/urdf/2_ur5.urdf -o ${TWO_ARM_C} -n "kavrakilab")
    set_source_files_properties(${TWO_ARM_C} PROPERTIES GENERATED TRUE)
    add_custom_target(two_arm_target ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/2_ur5.urdf)
    add_library(${TWO_ARM_LIB} SHARED ${TWO_ARM_C})

    ## Add cmake target dependencies of the executable/library
    ## as an example, message headers may need to be generated before nodes
    add_dependencies(${ONE_ARM_LIB} ${ONE_ARM_C})
    add_dependencies(${TWO_ARM_LIB} two_arm_target)
else()
    message(STATUS "Complied scenegraph libaries (for Amino) will not be generated.")
endif()

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_description_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${ONE_ARM_LIB} ${TWO_ARM_LIB}
        RUNTIME DESTINATION lib
        LIBRARY DESTINATION lib)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ur_description ur_description_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
