cmake_minimum_required(VERSION 3.8)
project(gimbal_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

include_directories(include)

add_executable(gimbal_control src/gimbal_control.cpp)
ament_target_dependencies(gimbal_control rclcpp std_srvs Eigen3 std_msgs)
target_link_libraries(gimbal_control
    MAVSDK::mavsdk
)
install(TARGETS gimbal_control DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
