cmake_minimum_required(VERSION 3.8)
project(negotiated)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(negotiated_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/negotiated_publisher.cpp
  src/negotiated_subscription.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${negotiated_interfaces_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(negotiated_interfaces rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_gtest(test_combinations test/test_combinations.cpp)
  target_link_libraries(test_combinations ${PROJECT_NAME})

  ament_add_gtest(test_negotiated_subscription test/test_negotiated_subscription.cpp)
  target_link_libraries(test_negotiated_subscription
    rclcpp::rclcpp
    ${PROJECT_NAME}
    ${negotiated_interfaces_TARGETS}
    ${std_msgs_TARGETS}
  )

  ament_add_gtest(test_negotiated_publisher test/test_negotiated_publisher.cpp)
  target_link_libraries(test_negotiated_publisher
    rclcpp::rclcpp
    ${PROJECT_NAME}
    ${negotiated_interfaces_TARGETS}
    ${std_msgs_TARGETS}
  )
endif()

ament_package()
