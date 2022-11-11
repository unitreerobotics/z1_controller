cmake_minimum_required(VERSION 3.0)
project(z1_controller LANGUAGES CXX)
add_definitions(-std=c++14)
set(PACKAGE_NAME z1_controller)
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3")

# ----------------------options----------------------
set(COMMUNICATION UDP)            # UDP
# set(COMMUNICATION ROS)          # ROS


# ----------------------configurations----------------------
# COMMUNICATION
if(${COMMUNICATION} STREQUAL "UDP")
    set(SIMULATION OFF)
elseif(${COMMUNICATION} STREQUAL "ROS")
    add_definitions(-DCOMPILE_WITH_SIMULATION)
    set(SIMULATION ON)
else()
    message(FATAL_ERROR "[CMake ERROR] The COMMUNICATION is error")
endif()


if(SIMULATION)
    find_package(gazebo REQUIRED)
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
    )

    link_directories(
        ${GAZEBO_LIBRARY_DIRS}
    )

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
endif()

set(EIGEN_PATH /usr/include/eigen3)
include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${EIGEN_PATH}
)

link_directories(lib)

# ----------------------add executable----------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../build)                # EXECUTABLE:CMAKE_RUNTIME_OUTPUT_DIRECTORY
add_executable(z1_ctrl main.cpp)
target_link_libraries(z1_ctrl libZ1_${COMMUNICATION}_Linux64.so ${catkin_LIBRARIES} )
