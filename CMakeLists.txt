cmake_minimum_required(VERSION 2.8.3)
project(ipa_visual_free_space_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure 
  std_msgs
  image_transport
  cv_bridge
  roscpp
  sensor_msgs
  stereo_msgs
  pcl_ros
)

generate_dynamic_reconfigure_options(cfg/free_space_detection_node.cfg )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    dynamic_reconfigure
    std_msgs
    image_transport
    cv_bridge
    roscpp
    sensor_msgs
    stereo_msgs
    pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(free_space_detection_node ros/src/free_space_detection_node_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(free_space_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(free_space_detection_node
  ${catkin_LIBRARIES}
)

add_definitions(-std=c++0x)
target_link_libraries(free_space_detection_node
  ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS free_space_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
