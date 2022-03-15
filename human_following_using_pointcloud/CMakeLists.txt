cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_preprocessing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

# link_directories(
#     include
#     ${OpenCV_LIBRARY_DIRS}
#     ${PCL_LIBRARY_DIRS}
#   )
  
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


# add_executable(cluster src/cluster.cpp)
# target_link_libraries(cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(raw_data src/raw_data.cpp)
target_link_libraries(raw_data ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ransac src/ransac.cpp)
target_link_libraries(ransac ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(old_ransac src/old_ransac.cpp)
target_link_libraries(old_ransac ${catkin_LIBRARIES} ${PCL_LIBRARIES})




