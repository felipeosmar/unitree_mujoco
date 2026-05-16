cmake_minimum_required(VERSION 3.12)
project(platform_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(pulse_platform pulse_platform.cpp)
target_link_libraries(pulse_platform PRIVATE unitree_sdk2 pthread)

add_executable(keyboard_platform keyboard_platform.cpp)
target_link_libraries(keyboard_platform PRIVATE unitree_sdk2 pthread)
