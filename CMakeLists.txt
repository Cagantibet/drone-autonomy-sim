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
  src/world2d.cpp
  src/collision.cpp
  src/rrt.cpp
  src/rasterize.cpp
  src/smoothing.cpp
  src/pid.cpp
  src/follower.cpp
  src/tracking_metrics.cpp
)
target_include_directories(drone_core PUBLIC include)
target_link_libraries(drone_core PUBLIC Eigen3::Eigen)
target_compile_options(drone_core PRIVATE -Wall -Wextra -Wpedantic)

# ---- apps ----
add_executable(plan_path apps/plan_path.cpp)
target_link_libraries(plan_path PRIVATE drone_core)
target_compile_options(plan_path PRIVATE -Wall -Wextra -Wpedantic)

add_executable(plan_rrt apps/plan_rrt.cpp)
target_link_libraries(plan_rrt PRIVATE drone_core)
target_compile_options(plan_rrt PRIVATE -Wall -Wextra -Wpedantic)

add_executable(compare_planners apps/compare_planners.cpp)
target_link_libraries(compare_planners PRIVATE drone_core)
target_compile_options(compare_planners PRIVATE -Wall -Wextra -Wpedantic)

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
    tests/test_world2d.cpp
    tests/test_collision.cpp
    tests/test_rrt.cpp
    tests/test_rasterize.cpp
    tests/test_smoothing.cpp
    tests/test_pid.cpp
    tests/test_follower.cpp
    tests/test_tracking_metrics.cpp
  )
  target_link_libraries(drone_tests PRIVATE drone_core GTest::gtest_main)
  target_compile_definitions(drone_tests PRIVATE
    DRONE_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/maps")

  include(GoogleTest)
  gtest_discover_tests(drone_tests)
endif()
