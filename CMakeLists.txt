cmake_minimum_required(VERSION 3.10)
project(lite3_custom_gait)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-w)

# Build platform: x86 (default) or arm
set(BUILD_PLATFORM "x86" CACHE STRING "Target platform: x86 or arm")

if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER   "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
endif()

# ── ROS2 dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(drdds        REQUIRED)

# ── Include paths ──────────────────────────────────────────────────────────────
include_directories(
  include
)

# ── Executable ─────────────────────────────────────────────────────────────────
add_executable(gait_runner src/main.cpp)

ament_target_dependencies(gait_runner
  rclcpp
  drdds
)

target_link_libraries(gait_runner
  -lpthread
  -lm
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS gait_runner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
