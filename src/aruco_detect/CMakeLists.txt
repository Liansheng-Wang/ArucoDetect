cmake_minimum_required(VERSION 3.0.2)
project(aruco_detect)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  tf
  aruco
  aruco_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(find_mark_node src/find_mark.cpp)
add_dependencies(find_mark_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_mark_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(create_map_node src/create_map.cpp)
target_link_libraries(create_map_node ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(record_mark_node src/record_mark.cpp)
target_link_libraries(record_mark_node ${catkin_LIBRARIES})
