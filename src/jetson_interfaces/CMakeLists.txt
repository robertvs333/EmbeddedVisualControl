cmake_minimum_required(VERSION 3.8)
project(jetson_interfaces)

# Default to C99 and C++14
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Generate the custom message
# This block tells ROS to look in the 'msg' folder for your file
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DualImage.msg"
  DEPENDENCIES sensor_msgs
)

# 3. Testing (Standard boilerplate)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. Finalize the package
ament_package()