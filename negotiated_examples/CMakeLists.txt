cmake_minimum_required(VERSION 3.8)
project(negotiated_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(negotiated REQUIRED)
find_package(std_msgs REQUIRED)

add_library(neg_sub_example1_node SHARED
  src/neg_sub_example1.cpp
)
target_link_libraries(neg_sub_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(neg_sub_example1_node
  PLUGIN "negotiated_examples::NegotiatedSubExample1"
  EXECUTABLE neg_sub_example1)

add_library(neg_sub_example2_node SHARED
  src/neg_sub_example2.cpp
)
target_link_libraries(neg_sub_example2_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(neg_sub_example2_node
  PLUGIN "negotiated_examples::NegotiatedSubExample2"
  EXECUTABLE neg_sub_example2)

add_library(neg_pub_example1_node SHARED
  src/neg_pub_example1.cpp
)
target_link_libraries(neg_pub_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(neg_pub_example1_node
  PLUGIN "negotiated_examples::NegotiatedPubExample1"
  EXECUTABLE neg_pub_example1)

install(
  TARGETS neg_sub_example1_node neg_sub_example2_node neg_pub_example1_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(negotiated_interfaces negotiated rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
