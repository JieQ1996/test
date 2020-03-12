cmake_minimum_required(VERSION 2.8.3)
project(get_line)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES get_line
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
  DEPENDS  EIGEN3 PCL 
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/getLP_node.cpp src/getLP.cpp)
add_executable(makeP src/makePoints.cpp)

target_link_libraries(${PROJECT_NAME}_node
		${PCL_LIBRARIES}
		${catkin_LIBRARIES}
)
target_link_libraries(makeP
		${PCL_LIBRARIES}
		${catkin_LIBRARIES}
		${EIGEN3_LIBS}
)
