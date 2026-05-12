find_package(GTest REQUIRED)
include(GoogleTest)
get_target_property(GTEST_IMPORTED_INCLUDES GTest::gtest INTERFACE_INCLUDE_DIRECTORIES)

add_executable(env_tests
    test_environment.cpp
    test_game.cpp
    test_lidar.cpp
    test_robot.cpp
)

target_link_libraries(env_tests
    GTest::gtest
    GTest::gtest_main
    robot_lib
)

target_include_directories(env_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${GTEST_INCLUDE_DIRS}
    ${GTEST_IMPORTED_INCLUDES}
)

target_compile_definitions(env_tests PRIVATE
    TEST_RESOURCES_DIR="${CMAKE_SOURCE_DIR}/resources"
)

add_test(NAME env_tests COMMAND env_tests)
