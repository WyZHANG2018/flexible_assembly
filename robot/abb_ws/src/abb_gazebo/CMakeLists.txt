cmake_minimum_required(VERSION 2.8.3)
project(abb_gazebo)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_msgs
  gazebo_dev
  gazebo_plugins
  gazebo_ros_control
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  geometry_msgs
  sensor_msgs
  cv_bridge
)

find_package(gazebo REQUIRED)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    gazebo_msgs
    gazebo_dev
    gazebo_plugins
    gazebo_ros_control
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    geometry_msgs
    sensor_msgs
    cv_bridge
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
    EIGEN3
    roscpp
    gazebo_ros

)

add_library(${PROJECT_NAME} src/icosahedron.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(grasp_pose src/grasp_pose.cpp)
target_link_libraries(grasp_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME})

#add_library(${PROJECT_NAME} src/add_remove_joint_plugin.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_executable(grasp_shoot src/grasp_shoot.cpp)
#target_link_libraries(grasp_shoot ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_executable(sub src/sub.cpp)
#target_link_libraries(sub ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_executable(complet src/complet.cpp)
#target_link_libraries(complet ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_executable(nuts src/nuts.cpp)
#target_link_libraries(nuts ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

#add_executable(image src/image.cpp)
#target_link_libraries(image ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
