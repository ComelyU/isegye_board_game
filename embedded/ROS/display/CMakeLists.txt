cmake_minimum_required(VERSION 3.5)
project(display LANGUAGES CXX)

# ROS 헤더 파일이 있는 경로를 포함합니다.
include_directories(
    ${ament_cmake_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Find ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(display
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

# Link ROS libraries
target_link_libraries(display PRIVATE
    Qt5::Widgets
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${ament_cmake_LIBRARIES}
)