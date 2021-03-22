# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(jointSpaceIterator)

find_package(YCM ${YCM_REQUIRED_VERSION} REQUIRED)
find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)
find_package(gazebo REQUIRED)
find_package(GazeboYARPPlugins REQUIRED)

add_library(${PROJECT_NAME}-collisionDetector SHARED src/collisionPlugin.cc include/collisionPlugin.hh )
target_compile_definitions(${PROJECT_NAME}-collisionDetector PRIVATE _USE_MATH_DEFINES)
target_include_directories(${PROJECT_NAME}-collisionDetector PRIVATE include  ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-collisionDetector PRIVATE ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES} GazeboYARPPlugins::gazebo_yarp_lib_common GazeboYARPPlugins::gazebo_yarp_singleton )
install(TARGETS ${PROJECT_NAME}-collisionDetector LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)
