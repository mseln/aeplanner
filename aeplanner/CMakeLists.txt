cmake_minimum_required(VERSION 2.8.3)
project(aeplanner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  rqt_gui_cpp
  rqt_gui
  tf
  tf2
  genmsg 
  actionlib_msgs 
  actionlib
  nodelet
  std_msgs

  kdtree
  pigain
  dynamic_reconfigure
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

add_message_files(
  DIRECTORY msg 
  FILES Node.msg
)
add_action_files(
  DIRECTORY action 
  FILES aeplanner.action
)
add_service_files(
  DIRECTORY srv
  FILES Reevaluate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  LIBRARIES aeplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs tf kdtree actionlib_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(aeplanner src/aeplanner_nodelet.cpp src/aeplanner.cpp src/aeplanner_viz.cpp src/param.cpp)
target_link_libraries(aeplanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(aeplanner
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

add_executable(aeplanner_node src/aeplanner_node.cpp src/aeplanner.cpp src/aeplanner_viz.cpp src/param.cpp)
target_link_libraries(aeplanner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(aeplanner_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
