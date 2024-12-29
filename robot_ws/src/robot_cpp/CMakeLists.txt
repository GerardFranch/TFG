cmake_minimum_required(VERSION 3.8)
project(robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publicador_simple src/publicador_simple.cpp)
ament_target_dependencies(publicador_simple rclcpp std_msgs)

add_executable(suscriptor_simple src/suscriptor_simple.cpp)
ament_target_dependencies(suscriptor_simple rclcpp std_msgs)

install(TARGETS 
  publicador_simple
  suscriptor_simple
  DESTINATION lib/robot_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
