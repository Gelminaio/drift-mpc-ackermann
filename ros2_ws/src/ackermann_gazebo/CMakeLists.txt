cmake_minimum_required(VERSION 3.8)
project(ackermann_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(sim_car SHARED src/sim_car.cpp)
target_compile_features(sim_car PUBLIC cxx_std_17)
target_link_libraries(sim_car
  ament_index_cpp::ament_index_cpp
  ${ackermann_msgs_TARGETS}
  gz-plugin::register
  gz-sim::gz-sim
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  yaml-cpp::yaml-cpp
)

install(TARGETS sim_car LIBRARY DESTINATION lib)
install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
