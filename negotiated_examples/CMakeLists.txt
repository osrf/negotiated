cmake_minimum_required(VERSION 3.8)
project(negotiated_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(negotiated REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_library(negotiated_sub_example1_node SHARED
  src/negotiated_sub_example1.cpp
)
target_link_libraries(negotiated_sub_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_sub_example1_node
  PLUGIN "negotiated_examples::NegotiatedSubExample1"
  EXECUTABLE negotiated_sub_example1)

add_library(negotiated_sub_example2_node SHARED
  src/negotiated_sub_example2.cpp
)
target_link_libraries(negotiated_sub_example2_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_sub_example2_node
  PLUGIN "negotiated_examples::NegotiatedSubExample2"
  EXECUTABLE negotiated_sub_example2)

add_library(negotiated_sub_example3_node SHARED
  src/negotiated_sub_example3.cpp
)
target_link_libraries(negotiated_sub_example3_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_sub_example3_node
  PLUGIN "negotiated_examples::NegotiatedSubExample3"
  EXECUTABLE negotiated_sub_example3)

add_library(negotiated_pub_example1_node SHARED
  src/negotiated_pub_example1.cpp
)
target_link_libraries(negotiated_pub_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_pub_example1_node
  PLUGIN "negotiated_examples::NegotiatedPubExample1"
  EXECUTABLE negotiated_pub_example1)

add_library(negotiated_pub_sub_pair_example1_node SHARED
  src/negotiated_pub_sub_pair_example1.cpp
)
target_link_libraries(negotiated_pub_sub_pair_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_pub_sub_pair_example1_node
  PLUGIN "negotiated_examples::NegotiatedPubSubPairExample1"
  EXECUTABLE negotiated_pub_sub_pair_example1)

add_library(negotiated_pub_type_adapt_example1_node SHARED
  src/negotiated_pub_type_adapt_example1.cpp
)
target_link_libraries(negotiated_pub_type_adapt_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_pub_type_adapt_example1_node
  PLUGIN "negotiated_examples::NegotiatedPubTypeAdaptExample1"
  EXECUTABLE negotiated_pub_type_adapt_example1)

add_library(negotiated_sub_type_adapt_example1_node SHARED
  src/negotiated_sub_type_adapt_example1.cpp
)
target_link_libraries(negotiated_sub_type_adapt_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_sub_type_adapt_example1_node
  PLUGIN "negotiated_examples::NegotiatedSubTypeAdaptExample1"
  EXECUTABLE negotiated_sub_type_adapt_example1)

add_executable(negotiated_lifecycle_pub
  src/negotiated_lifecycle_pub.cpp)
target_link_libraries(negotiated_lifecycle_pub
  negotiated::negotiated
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
)

add_library(negotiated_pub_compatible_example1_node SHARED
  src/negotiated_pub_compatible_example1.cpp
)
target_link_libraries(negotiated_pub_compatible_example1_node
  negotiated::negotiated
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${std_msgs_TARGETS}
)
rclcpp_components_register_node(negotiated_pub_compatible_example1_node
  PLUGIN "negotiated_examples::NegotiatedPubCompatibleExample1"
  EXECUTABLE negotiated_pub_compatible_example1)

install(
  TARGETS negotiated_sub_example1_node
          negotiated_sub_example2_node
          negotiated_sub_example3_node
          negotiated_pub_example1_node
          negotiated_pub_sub_pair_example1_node
          negotiated_pub_type_adapt_example1_node
          negotiated_sub_type_adapt_example1_node
          negotiated_pub_compatible_example1_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS negotiated_lifecycle_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(negotiated rclcpp rclcpp_lifecycle std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
