cmake_minimum_required(VERSION 3.16)
project(robot_assignment)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(Fontconfig REQUIRED)
find_package(GTest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(robot_lib
    src/Environment.cpp
    src/Game.cpp
    src/Lidar.cpp
    src/Robot.cpp
    src/WebServer.cpp
    src/Waste.cpp
    src/parser/Parser.cpp
)

target_include_directories(robot_lib PUBLIC include)
target_link_libraries(robot_lib PRIVATE ${OpenCV_LIBS})

add_library(opencv_visualization
    src/Canvas.cpp
    src/OpenCvSceneRenderer.cpp
)

target_include_directories(opencv_visualization PUBLIC include)
target_link_libraries(opencv_visualization PUBLIC robot_lib ${OpenCV_LIBS} Fontconfig::Fontconfig)

add_executable(program main.cpp)
target_sources(program PRIVATE
    src/GameNode.cpp
    src/RobotNodes.cpp
    src/controls/Controls.cpp
)
target_link_libraries(program robot_lib opencv_visualization)
target_link_libraries(program
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
)


enable_testing()
add_subdirectory(tests)
# Toto nemazat, v pripade ak to zmazete, github classroom nebude fungovat
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/hidden_tests/CMakeLists.txt)
    add_subdirectory(hidden_tests)
endif()
