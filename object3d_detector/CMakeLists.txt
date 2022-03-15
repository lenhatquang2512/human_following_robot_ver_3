cmake_minimum_required(VERSION 2.8.3)
project(object3d_detector)

# set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
                              ${PCL_INCLUDE_DIRS})

                              
add_definitions(${PCL_DEFINITIONS})


catkin_package()

catkin_package(
  INCLUDE_DIRS include
  #CATKIN_DEPENDS bayes_tracking
  )

### Offline version
# add_executable(object3d_detector src/object3d_detector.cpp src/svm.cpp)
add_executable(object3d_detector src/object3d_detector.cpp src/svm_fix.cpp)
# add_executable(object3d_detector src/object3d_detector_cluster.cpp)
# add_executable(object3d_detector src/quang_cluster.cpp)
target_link_libraries(object3d_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector ${catkin_EXPORTED_TARGETS})
endif()

