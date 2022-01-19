cmake_minimum_required(VERSION 3.8)
project(negotiated)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(negotiated_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
  ${negotiated_interfaces_TARGETS}
)

add_executable(neg_sub_example1
  src/neg_sub_example1.cpp
)
target_include_directories(neg_sub_example1 PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(neg_sub_example1
  rclcpp::rclcpp
  ${std_msg_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(neg_pub_example1
  src/neg_pub_example1.cpp
)
target_include_directories(neg_pub_example1 PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(neg_pub_example1
  rclcpp::rclcpp
  ${std_msg_LIBRARIES}
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(negotiated_interfaces rclcpp)

install(TARGETS
  neg_sub_example1
  neg_pub_example1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
