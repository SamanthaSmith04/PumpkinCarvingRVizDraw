cmake_minimum_required(VERSION 3.8)
project(pumpkin_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pumpkin_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare the executable
add_executable(motion_planning_client src/test_client.cpp)

# Link dependencies
ament_target_dependencies(motion_planning_client
  rclcpp
  pumpkin_msgs
  geometry_msgs
  yaml-cpp
)

target_include_directories(motion_planning_client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motion_planning_client
    yaml-cpp
)   

# Install executables
install(TARGETS
  motion_planning_client
  DESTINATION lib/${PROJECT_NAME}
)

# Handle testing dependencies if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Complete the package
ament_package()
