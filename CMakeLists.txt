cmake_minimum_required(VERSION 3.16)
project(robot_assignment)

set(CMAKE_CXX_STANDARD 20)

# Doplnte ostate potrebne parametre

include(FetchContent)
find_package(OpenCV REQUIRED)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

FetchContent_MakeAvailable(googletest)
include(GoogleTest)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(main
    main.cpp
    src/Enviroment.cpp
    src/Lidar.cpp
    src/Canvas.cpp
)

add_executable(tests
    src/tests/test_environment.cpp
    src/tests/test_lidar.cpp
    src/Enviroment.cpp
    src/Lidar.cpp
)

target_link_libraries(main
    PRIVATE
    ${OpenCV_LIBS}
)

target_include_directories(main
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(tests
    PRIVATE
    GTest::gtest
    GTest::gtest_main
    ${OpenCV_LIBS}
)

target_include_directories(tests
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

gtest_discover_tests(tests)

# Toto nemazat, v pripade ak to zmazete, github classroom nebude fungovat
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/hidden_tests/CMakeLists.txt)
    add_subdirectory(hidden_tests)
endif()
