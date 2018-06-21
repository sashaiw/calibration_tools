cmake_minimum_required(VERSION 2.8.3)
project(calibration_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  image_geometry
  aruco
)

catkin_package()

find_package(PCL 1.8 EXACT REQUIRED)
find_package(OpenCV 2 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(calibration_wizard src/calibration_wizard.cpp src/find_chessboard.h src/find_chessboard.cpp)
target_link_libraries(calibration_wizard ${OpenCV_LIBRARIES})
target_link_libraries(calibration_wizard ${catkin_LIBRARIES})

add_executable(icp src/icp.cpp)
target_link_libraries(icp ${PCL_LIBRARIES})
target_link_libraries(icp ${catkin_LIBRARIES})