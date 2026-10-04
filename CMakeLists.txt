cmake_minimum_required(VERSION 3.8)
project(auto_aim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(auto_aim_node
  src/armor_detector.cpp
  src/plate_tracker.cpp
  src/aim_solver.cpp
  src/serial_controller.cpp
  src/auto_aim_node.cpp
  src/main.cpp
)

target_include_directories(auto_aim_node BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(auto_aim_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(auto_aim_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

install(TARGETS auto_aim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
