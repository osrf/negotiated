cmake_minimum_required(VERSION 3.8)
project(negotiated_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(negotiated REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(neg_sub_example1
  src/neg_sub_example1.cpp
)
target_link_libraries(neg_sub_example1
  negotiated::negotiated
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(neg_sub_example2
  src/neg_sub_example2.cpp
)
target_link_libraries(neg_sub_example2
  negotiated::negotiated
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(neg_pub_example1
  src/neg_pub_example1.cpp
)
target_link_libraries(neg_pub_example1
  negotiated::negotiated
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

ament_export_dependencies(negotiated_interfaces negotiated rclcpp std_msgs)

install(TARGETS
  neg_sub_example1
  neg_sub_example2
  neg_pub_example1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
