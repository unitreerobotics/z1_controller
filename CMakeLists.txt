cmake_minimum_required(VERSION 3.0)
project(z1_controller LANGUAGES CXX)
set(PACKAGE_NAME z1_controller)
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++14 -pthread")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    include
)

link_directories(lib)
# ----------------------add executable----------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
add_executable(z1_ctrl main.cpp)
target_link_libraries(z1_ctrl libZ1_${CMAKE_HOST_SYSTEM_PROCESSOR}.so)

find_package(catkin)
if(${catkin_FOUND})
    add_subdirectory(sim)
endif()