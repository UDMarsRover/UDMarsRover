cmake_minimum_required(VERSION 3.8)
project(rover_touch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Try Qt6 first, fall back to Qt5
set(QT_MIN_VERSION 5.12)
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
  set(QT_WIDGETS_LIB Qt6::Widgets)
else()
  find_package(Qt5 ${QT_MIN_VERSION} COMPONENTS Widgets REQUIRED)
  set(QT_VERSION_MAJOR 5)
  set(QT_WIDGETS_LIB Qt5::Widgets)
endif()

# Enable Qt automatic processing of MOC / UIC / RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Collect sources (currently only main.cpp; UI/implementation files can be added later)
set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/mainwindow.cpp
  src/mainwindow.h
  src/ui_node.cpp
  src/touch_ui_node.h
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS})

target_link_libraries(${PROJECT_NAME}_node
  ${QT_WIDGETS_LIB}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
