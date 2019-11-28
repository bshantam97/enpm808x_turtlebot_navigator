cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_navigator)

add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  octomap_ros
  octomap_msgs
  sensor_msgs
  geometry_msgs
  rostest
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  $(OCTOMAP_INCLUDE_DIRS)
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap
)

set(LINK_LIBS
    ${OCTOMAP_LIBRARIES}
    ${catkin_LIBRARIES}
)

#add_executable(turtlebot_navigator src/main.cpp)
#target_link_libraries(turtlebot_navigator ${catkin_LIBRARIES})



if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #add_rostest_gtest(turtlebot_navigator launch/test.launch test/main.cpp)
  #target_link_libraries(turtlebot_navigator ${catkin_LIBRARIES})
endif()
