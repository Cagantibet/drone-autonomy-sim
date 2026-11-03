cmake_minimum_required(VERSION 3.16)
project(drone_autonomy_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(Eigen3 REQUIRED NO_MODULE)

# ---- core library ----
add_library(drone_core
  src/drone.cpp
  src/grid.cpp
  src/astar.cpp
)
target_include_directories(drone_core PUBLIC include)
target_link_libraries(drone_core PUBLIC Eigen3::Eigen)
target_compile_options(drone_core PRIVATE -Wall -Wextra -Wpedantic)

# ---- tests ----
option(DRONE_BUILD_TESTS "Build unit tests" ON)
if(DRONE_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(drone_tests
    tests/test_drone.cpp
    tests/test_grid.cpp
    tests/test_astar.cpp
  )
  target_link_libraries(drone_tests PRIVATE drone_core GTest::gtest_main)
  target_compile_definitions(drone_tests PRIVATE
    DRONE_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/maps")

  include(GoogleTest)
  gtest_discover_tests(drone_tests)
endif()
