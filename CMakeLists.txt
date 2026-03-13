cmake_minimum_required(VERSION 3.16)
project(robot_assignment)

set(CMAKE_CXX_STANDARD 20)

# Doplnte ostate potrebne parametre



# Toto nemazat, v pripade ak to zmazete, github classroom nebude fungovat
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/hidden_tests/CMakeLists.txt)
    add_subdirectory(hidden_tests)
endif()
