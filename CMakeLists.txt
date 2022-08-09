##          Copyright Emil Fresk 2018.
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE.md or copy at
##          http://www.boost.org/LICENSE_1_0.txt)

cmake_minimum_required(VERSION 3.5)
project(px4_comm)


## Compile as C++14
add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${mavros_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node
  src/px4_comm_node.cpp
  src/px4_comm.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp mav_msgs mavros mavros_msgs)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
 )

install (DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
