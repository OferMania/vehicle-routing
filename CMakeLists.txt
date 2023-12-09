# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.14)

# Name of the project
project(VehicleRouting)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Compilation options
add_compile_options(-Wall -g)

# Create a few variables for the folder names, so they are easier to rename in
# the future
set(SRC_DIR src)

# Add all the source files needed to build the executable
add_executable(
  VehicleRouting
  ${SRC_DIR}/main.cpp
  ${SRC_DIR}/graph.cpp
  ${SRC_DIR}/evaluate_shared.cpp
)

enable_testing()

add_executable(
  VehicleRoutingTests
  ${SRC_DIR}/main_tests.cpp
  ${SRC_DIR}/graph_tests.cpp
  ${SRC_DIR}/graph.cpp
)

target_link_libraries(
  VehicleRoutingTests
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(VehicleRoutingTests)