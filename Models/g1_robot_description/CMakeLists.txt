cmake_minimum_required(VERSION 3.8)
project(g1_robot_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package только для C++ зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)  # Добавьте если используете

# Для launch_ros используем ament_cmake, так как это Python-пакет
# НЕ ИСПОЛЬЗУЙТЕ find_package для launch_ros!

# Установка ресурсов (URDF и мешей)
install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()