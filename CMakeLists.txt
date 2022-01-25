cmake_minimum_required(VERSION 3.0.2)
project(ros-pcl-tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#find_package(PCL REQUIRED)
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ros-pcl-tutorials
  #CATKIN_DEPENDS eigen_conversions geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tra src/transforms.cpp)
target_link_libraries(tra ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(voxel src/voxel_grid.cpp)
target_link_libraries(voxel ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(statistical src/statistical_removal.cpp)
target_link_libraries(statistical ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(kd_tree src/kdtree_search.cpp)
target_link_libraries(kd_tree ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(icp src/icp.cpp)
target_link_libraries(icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(resampling src/resampling.cpp)
target_link_libraries(resampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})