cmake_minimum_required(VERSION 2.8.3)
project(rtt-gazebo-lwr4plus-sim)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(gazebo REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
#find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(RCI REQUIRED)
message(STATUS "RCI version: ${RCI_VERSION}")	
include_directories(BEFORE SYSTEM ${RCI_INCLUDE_DIRS})	
add_definitions(${RCI_CFLAGS})
link_directories(${RCI_LIBRARY_DIR})

find_package(RSC REQUIRED)
message(STATUS "RSC version: ${RSC_VERSION}")	
include_directories(BEFORE SYSTEM ${RSC_INCLUDE_DIRS})	
add_definitions(${RSC_CFLAGS})
link_directories(${RSC_LIBRARY_DIR})

find_package(NemoMath REQUIRED)
message(STATUS "NemoMath version: ${NemoMath_VERSION}")	
include_directories(BEFORE SYSTEM ${NemoMath_INCLUDE_DIRS})	
add_definitions(${NemoMath_CFLAGS})
link_directories(${NemoMath_LIBRARY_DIR})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
	find_path("/usr/include" tinyxml.h)
	find_library(TINYXML_LIBRARY NAMES tinyxml)
	set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/rtt-gazebo-lwr4plus-sim.cpp src/rtt-gazebo-lwr4plus-sim_hooks.cpp src/parsertools/KDLParser.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   #${kdl_conversions_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES} 
   #${OROCOS-RTT_TYPEKIT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${NemoMath_LIBRARIES}
   ${RSC_LIBRARIES}
   ${RCI_LIBRARIES}
   ${TINYXML_LIBRARIES}
)

orocos_generate_package(INCLUDE_DIRS include)

#install(DIRECTORY scripts/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
#  PATTERN ".svn" EXCLUDE)
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE)
