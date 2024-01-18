cmake_minimum_required(VERSION 3.13)
project(omega)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs trajectory_msgs turtlebot3_msgs image_transport cv_bridge)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs trajectory_msgs turtlebot3_msgs image_transport cv_bridge
  DEPENDS Eigen3 OpenCV
)

###########
## Build ##
###########

add_executable(omega
  src/arm.cpp
  src/ball_tracker.cpp
  src/camera.cpp
  src/config.cpp
  src/gripper.cpp
  src/omega.cpp
  src/robot_tracker.cpp
  src/timer.cpp
  src/wheels.cpp
)
target_include_directories(omega PUBLIC include ${catkin_INCLUDE_DIRS})
target_include_directories(omega PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(omega PUBLIC ${catkin_LIBRARIES})
target_link_libraries(omega PUBLIC ${OpenCV_LIBS})
target_link_libraries(omega PUBLIC Eigen3::Eigen)

add_dependencies(omega ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS omega
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE
)