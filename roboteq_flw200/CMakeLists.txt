cmake_minimum_required(VERSION 3.8)
project(roboteq_flw200)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(dependencies "ament_index_cpp" "geometry_msgs" "rclcpp" "std_msgs")


include_directories(include/roboteq_flw200)
add_executable(subscriber src/subscriber.cpp)
add_executable(roboteq_flw200 src/ReadOpticalSensor.cpp src/ceSerial.cpp)
ament_target_dependencies(roboteq_flw200 rclcpp std_msgs geometry_msgs)
ament_target_dependencies(subscriber rclcpp std_msgs geometry_msgs)
target_include_directories(roboteq_flw200 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboteq_flw200 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS roboteq_flw200 subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
