cmake_minimum_required(VERSION 3.18)
project(motion_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs shape_msgs visualization_msgs
  moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface
  moveit_visual_tools rviz_visual_tools geometric_shapes
)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp geometry_msgs shape_msgs visualization_msgs
  moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface
  moveit_visual_tools rviz_visual_tools geometric_shapes
)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

foreach(generator data_generator_node data_generator_path_obs data_generator_app_scene)
  add_executable(${generator} src/${generator}.cpp)
  target_link_libraries(${generator} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
  if(catkin_EXPORTED_TARGETS)
    add_dependencies(${generator} ${catkin_EXPORTED_TARGETS})
  endif()
  install(TARGETS ${generator} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
