cmake_minimum_required(VERSION 2.8.3)
project(calibration_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

find_package(PCL 1.8 EXACT REQUIRED)

#set(OpenCV_DIR "/opt/ros/indigo")
#set(OpenCV_DIR "/opt/ros/indigo/include/opencv-3.1.0-dev")
#find_package(OpenCV 3.1.0 REQUIRED HINTS ${OpenCV_DIR} NO_DEFAULT_PATH)
#find_package(OpenCV 3.1.0 REQUIRED)
find_package(OpenCV 2 REQUIRED)

find_package(aruco REQUIRED)

include_directories(include)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calibration_tools
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#add_executable(find_aruco src/find_aruco.cpp)
#target_link_libraries(find_aruco ${OpenCV_LIBRARIES})
#target_link_libraries(find_aruco ${catkin_LIBRARIES})
#target_link_libraries(find_aruco ${aruco_LIBS})

add_executable(find_chessboard src/calibration_wizard.cpp src/find_chessboard.h src/find_chessboard.cpp)
target_link_libraries(find_chessboard ${OpenCV_LIBRARIES})
target_link_libraries(find_chessboard ${catkin_LIBRARIES})

add_executable(get_transforms src/get_transforms.cpp)
target_link_libraries(get_transforms ${PCL_LIBRARIES})
target_link_libraries(get_transforms ${catkin_LIBRARIES})

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${PCL_LIBRARIES})
target_link_libraries(image_converter ${OpenCV_LIBS})
target_link_libraries(image_converter ${catkin_LIBRARIES})
