find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(env_tests
    test_env.cpp
    test_lid.cpp
)

target_link_libraries(env_tests
    GTest::GTest
    GTest::Main
    robot_lib
)

target_include_directories(env_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
add_test(NAME env_tests COMMAND env_tests)