cmake_minimum_required(VERSION 3.5)
project(display LANGUAGES CXX)

include_directories("/opt/ros/foxy/include")

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
)