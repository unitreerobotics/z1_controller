cmake_minimum_required(VERSION 3.0)
project(z1_controller)
add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3")

# set(GRIPPER MASS3KG)          # 3kg
# set(GRIPPER NONE)             # No Gripper
set(GRIPPER UNITREE_GRIPPER)    # The gripper made by unitree

set(COMMUNICATION UDP)          # UDP
# set(COMMUNICATION ROS)        # ROS

# set(CTRL_PANEL KEYBOARD)        # control by keyboard
set(CTRL_PANEL SDK)                 # control by SDK
# set(CTRL_PANEL JOYSTICK)      # control by joystick

if(${COMMUNICATION} STREQUAL "UDP")
    add_definitions(-DUDP)
    set(SIMULATION OFF)
    set(REAL_ROBOT ON) 
elseif(${COMMUNICATION} STREQUAL "ROS")
    add_definitions(-DROS)
    set(SIMULATION ON)
    set(REAL_ROBOT OFF) 
else()
    message(FATAL_ERROR "[CMake ERROR] The COMMUNICATION is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
endif()

if(SIMULATION)
    add_definitions(-DCOMPILE_WITH_SIMULATION)
    add_definitions(-DRUN_ROS)
    find_package(gazebo REQUIRED)
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()


if(${GRIPPER} STREQUAL "NONE")
    add_definitions(-DNO_GRIPPER)
elseif(${GRIPPER} STREQUAL "AG95")
    add_definitions(-DAG95)
elseif(${GRIPPER} STREQUAL "MASS3KG")
    add_definitions(-DMASS3KG)
elseif(${GRIPPER} STREQUAL "UNITREE_GRIPPER")
    add_definitions(-DUNITREE_GRIPPER)
else()
    message(FATAL_ERROR "[CMake ERROR] The GRIPPER is error")
endif()


if(${CTRL_PANEL} STREQUAL "KEYBOARD")
    add_definitions(-DCTRL_BY_KEYBOARD)
elseif(${CTRL_PANEL} STREQUAL "JOYSTICK")
    add_definitions(-DCTRL_BY_JOYSTICK)
elseif(${CTRL_PANEL} STREQUAL "SDK")
    add_definitions(-DCTRL_BY_SDK)
else()
    message(FATAL_ERROR "[CMake ERROR] The CTRL_PANEL is error")
endif()

if(SIMULATION)
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
    thirdparty
    ../z1_sdk/include
    ${Boost_INCLUDE_DIR}
    ${EIGEN_PATH}
)

link_directories(lib)

# # add_library
# file(GLOB_RECURSE ADD_SRC_LIST
#     src/*/*.cpp
#     src/*/*/*.cpp
#     thirdparty/*/src/*.cpp
#     thirdparty/*/src/*.cc
#     ../z1_sdk/src/*.cpp
# )

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)                  # SHARED:CMAKE_LIBRARY_OUTPUT_DIRECTORY   STATIC:CMAKE_ARCHIVE_OUTPUT_DIRECTORY
# add_library(Z1_${CTRL_PANEL}_${COMMUNICATION}_Linux64 SHARED ${ADD_SRC_LIST})
# target_link_libraries(Z1_${CTRL_PANEL}_${COMMUNICATION}_Linux64 ${catkin_LIBRARIES} rbdl -pthread)
# if(SIMULATION)
#     add_dependencies(Z1_${CTRL_PANEL}_${COMMUNICATION}_Linux64 ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# endif()


# add_executable and target_link_libraries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../build)                # EXECUTABLE:CMAKE_RUNTIME_OUTPUT_DIRECTORY
add_executable(z1_ctrl main.cpp)
target_link_libraries(z1_ctrl ${catkin_LIBRARIES} libUnitree_motor_SDK_Linux64_EPOLL.so libZ1_${CTRL_PANEL}_${COMMUNICATION}_Linux64.so)


# debug
if(DEBUG)
    find_package(PythonLibs REQUIRED)
    include_directories(${PYTHON_INCLUDE_DIRS})
    target_link_libraries(Z1_${CTRL_PANEL}_${COMMUNICATION}_Linux64 ${PYTHON_LIBRARIES})
endif()


# set(EXECUTABLE_OUTPUT_PATH /home/$ENV{USER}/)