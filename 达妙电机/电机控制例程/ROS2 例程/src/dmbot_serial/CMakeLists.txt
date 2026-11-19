cmake_minimum_required(VERSION 3.8)
project(dmbot_serial)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(motor
  src/protocol/damiao.cpp
  src/protocol/socketcan.cpp
)
target_include_directories(motor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(motor
  PUBLIC
    Threads::Threads
)

add_executable(test_motor_node src/test_motor_node.cpp)
target_link_libraries(test_motor_node motor)
ament_target_dependencies(test_motor_node
  rclcpp
  std_msgs
)

install(
  TARGETS motor test_motor_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(motor)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
