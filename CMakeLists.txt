cmake_minimum_required(VERSION 3.8)
project(square_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SquareInt.srv"
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(server src/service_main.cpp src/service.cpp)
ament_target_dependencies(server rclcpp)
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(client src/client_main.cpp src/client.cpp)
ament_target_dependencies(client rclcpp)
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  server
  client
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