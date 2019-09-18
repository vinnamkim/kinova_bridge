cmake_minimum_required(VERSION 3.5)

project(kinova_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

# find ROS 1 packages
set(cmake_extras_files cmake/find_ros1_package.cmake cmake/find_ros1_interface_packages.cmake)
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(WARNING "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
find_ros1_package(actionlib)
if(NOT ros1_roscpp_FOUND)
  message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package(
    CONFIG_EXTRAS ${cmake_extras_files}
  )
  return()
endif()

find_ros1_package(kinova_msgs)

# find ROS 1 packages with messages / services
include(cmake/find_ros1_interface_packages.cmake)
find_ros1_interface_packages(ros1_message_packages)

set(prefixed_ros1_message_packages "")
foreach(ros1_message_package ${ros1_message_packages})
  if(NOT "${ros1_message_package}" STREQUAL "nodelet")
    find_ros1_package(${ros1_message_package} REQUIRED)
    list(APPEND prefixed_ros1_message_packages "ros1_${ros1_message_package}")
  endif()
endforeach()

ament_package(
  CONFIG_EXTRAS ${cmake_extras_files}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "ros1_actionlib"
  "ros1_kinova_msgs"
  "ros1_roscpp"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION bin
)
