cmake_minimum_required(VERSION 2.8.3)
project(astra_ros)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

find_package(Astra REQUIRED)
find_package(PCL REQUIRED)

function(add_astra_sdk_dependency _targetname)
  if(NOT TARGET ${_targetname})
    message("add_sdk_dependency: There is no target named '${_targetname}'")
    return()
  endif()
  if(TARGET ${SDK_DEPENDENT_TARGET})
    add_dependencies(${_targetname} ${SDK_DEPENDENT_TARGET})
  endif()
endfunction()

## Declare ROS messages and services

## Generate added messages and services
## generate_messages(DEPENDENCIES std_msgs sensor_msgs)


set (_projname "astra_ros")

set (${_projname}_SOURCES
  src/main.cpp
  src/publisher.cpp
  src/publisher.h
  src/frameListener.h
  src/key_handler.h
  src/common.h
)


include_directories(
	${catkin_INCLUDE_DIRS}
	${ASTRA_INCLUDE_DIR} 
	${PCL_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package()

add_executable(${_projname} ${${_projname}_SOURCES})

set_target_properties(${_projname} PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}cpp-api")

target_link_libraries(${_projname} 
	${catkin_LIBRARIES}
	${ASTRA_LIBRARIES} 
	${PCL_LIBRARY_DIRS}
)

## set_debug_working_dir(${_projname} "${TargetDir}")

add_astra_sdk_dependency(${_projname})

## install_sample(${_projname})
