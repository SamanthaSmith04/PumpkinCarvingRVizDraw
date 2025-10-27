cmake_minimum_required(VERSION 3.19)
project(pumpkin_planner_app LANGUAGES CXX)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(motoros2_interfaces REQUIRED)
find_package(rviz_selection_3d)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(MPI REQUIRED)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
  set_target_properties(MPI::MPI_C PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MPI_C_INCLUDE_PATH}"
    INTERFACE_LINK_LIBRARIES "${MPI_C_LIBRARIES}"
  )
endif()
# Qt configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Source files
set(UIS
    src/rviz_draw_gui.ui
)

qt5_wrap_ui(UI_HEADERS ${UIS})

set(SRCS ${UI_HEADERS}
    src/pumpkin_widget.cpp
    include/pumpkin_widget.hpp
)

# Common dependencies for all targets
set(COMMON_DEPS
    ament_index_cpp
    rclcpp
    motoros2_interfaces
    yaml-cpp
    rviz_selection_3d
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# Shared library
add_library(rviz_draw_qt_gui_lib SHARED ${SRCS} ${UIS})

# Define HAS_MOTOMAN_SUPPORT if motoros2_interfaces was found
if(motoros2_interfaces_FOUND)
    target_compile_definitions(rviz_draw_qt_gui_lib PUBLIC HAS_MOTOMAN_SUPPORT)
    message(STATUS "Motoman support enabled")
else()
    message(STATUS "Motoman support disabled - motoros2_interfaces not found")
endif()

target_include_directories(rviz_draw_qt_gui_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

ament_target_dependencies(rviz_draw_qt_gui_lib ${COMMON_DEPS})

target_link_libraries(rviz_draw_qt_gui_lib
    Qt5::Widgets
    yaml-cpp
)

# Main executable
add_executable(rviz_draw_qt_gui src/main.cpp)
target_link_libraries(rviz_draw_qt_gui rviz_draw_qt_gui_lib)

# RViz plugin
add_library(rviz_draw_gui_panel SHARED src/pumpkin_rviz_panel.cpp)

target_include_directories(rviz_draw_gui_panel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rviz_draw_gui_panel 
    rviz_draw_qt_gui_lib
    rviz_common::rviz_common
    pluginlib::pluginlib
)

ament_target_dependencies(rviz_draw_gui_panel
    ${COMMON_DEPS}
    rviz_common
    pluginlib
)

# Installation
install(TARGETS 
    rviz_draw_qt_gui
    rviz_draw_qt_gui_lib
    rviz_draw_gui_panel
    EXPORT ${PROJECT_NAME}_targets
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Package export
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${COMMON_DEPS} rviz_common pluginlib Qt5)
pluginlib_export_plugin_description_file(rviz_common config/pumpkin_rviz_panel_plugin_description.xml)
ament_package()