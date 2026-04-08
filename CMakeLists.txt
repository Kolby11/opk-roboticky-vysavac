cmake_minimum_required(VERSION 3.16)
project(robot_assignment)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

add_library(robot_lib
    src/Environment.cpp
    src/Lidar.cpp
    src/Canvas.cpp
    src/Robot.cpp
)

target_include_directories(robot_lib PUBLIC include)
target_link_libraries(robot_lib PUBLIC ${OpenCV_LIBS})

add_executable(program main.cpp)
target_link_libraries(program robot_lib)

enable_testing()
add_subdirectory(tests)
# Toto nemazat, v pripade ak to zmazete, github classroom nebude fungovat
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/hidden_tests/CMakeLists.txt)
    add_subdirectory(hidden_tests)
endif()
