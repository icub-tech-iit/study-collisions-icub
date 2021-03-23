# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
set(PROJECTNAME jointSpaceIterator)
project(${PROJECTNAME})


set(YCM_REQUIRED_VERSION 0.11.0)
set(YARP_REQUIRED_VERSION 3.3.2)

# mandatory use of these packages
find_package(YCM ${YCM_REQUIRED_VERSION} REQUIRED)
find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)
find_package(Gazebo REQUIRED)
find_package(matioCpp REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH} matioCpp)

include(YarpIDL)
include(YarpInstallationHelpers)

file(GLOB source src/*.cpp )
file(GLOB header include/*.h )

source_group("Source files" FILES ${source})
source_group("Header files" FILES ${header})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${GAZEBO_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${source} ${header})

target_include_directories(${PROJECT_NAME} PRIVATE ${GAZEBO_INCLUDE_DIRS}  )
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} matioCpp::matioCpp )

install(TARGETS ${PROJECTNAME} DESTINATION bin)