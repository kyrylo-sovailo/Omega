cmake_minimum_required(VERSION 3.13)
project(omega)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

###########
## Build ##
###########

add_executable(omega src/omega.cpp src/config.cpp)
target_include_directories(omega PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(omega PUBLIC ${catkin_LIBRARIES})
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