cmake_minimum_required(VERSION 2.8.3)
project(missionpkg)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS 
	cv_bridge 
	image_transport 
	message_runtime 
	sensor_msgs
)

find_package(OpenCV)

set (CMAKE_CXX_STANDARD 11)

include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/el_det_fi.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(objdetnfollow src/objdetnfollow.cpp)
target_link_libraries(objdetnfollow ${catkin_LIBRARIES})

