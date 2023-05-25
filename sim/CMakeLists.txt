find_package(catkin REQUIRED COMPONENTS
  controller_manager
  genmsg
  joint_state_controller
  robot_state_publisher
  roscpp
  gazebo_ros
  std_msgs
  tf
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  sim
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
add_executable(sim_ctrl sim_ctrl.cpp IOROS.cpp)
target_link_libraries(sim_ctrl
  libZ1_${CMAKE_HOST_SYSTEM_PROCESSOR}.so
  ${catkin_LIBRARIES}
)