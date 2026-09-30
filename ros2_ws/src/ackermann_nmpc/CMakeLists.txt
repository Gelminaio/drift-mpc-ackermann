cmake_minimum_required(VERSION 3.8)
project(ackermann_nmpc)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ackermann_description REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# acados from source (docs/nmpc.md)
set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
if(NOT ACADOS_DIR)
  message(FATAL_ERROR "ACADOS_SOURCE_DIR is not set, see docs/nmpc.md")
endif()

# the solvers in C from ocp.py, with the parameters of vehicle_params.yaml built in
set(PARAMS ${ackermann_description_DIR}/../config/vehicle_params.yaml)
foreach(name nmpc drift)
  execute_process(
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate.py ${PARAMS} ${CMAKE_CURRENT_BINARY_DIR}/${name} ${name}
    RESULT_VARIABLE generate_result)
  if(generate_result)
    message(FATAL_ERROR "scripts/generate.py failed for ${name}")
  endif()
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${PARAMS}
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate.py
  ${CMAKE_CURRENT_SOURCE_DIR}/ackermann_nmpc/model.py
  ${CMAKE_CURRENT_SOURCE_DIR}/ackermann_nmpc/ocp.py)

# one node per solver
foreach(name nmpc drift)
  add_executable(${name}_node src/${name}_node.cpp)
  target_compile_features(${name}_node PUBLIC cxx_std_17)
  target_include_directories(${name}_node PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/${name}
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include)
  target_link_directories(${name}_node PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/${name} ${ACADOS_DIR}/lib)
  target_link_libraries(${name}_node
    acados_ocp_solver_${name} acados hpipm blasfeo
    ${ackermann_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros)
  # RPATH, not RUNPATH: libacados.so finds hpipm and blasfeo through it too
  set_target_properties(${name}_node PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${ACADOS_DIR}/lib")
  target_link_options(${name}_node PRIVATE -Wl,--disable-new-dtags)
  install(TARGETS ${name}_node DESTINATION lib/${PROJECT_NAME})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${name}/libacados_ocp_solver_${name}.so DESTINATION lib)
endforeach()

# the handbrake turn: the model in plain C++, no solver
add_executable(handbrake_node src/handbrake_node.cpp)
target_compile_features(handbrake_node PUBLIC cxx_std_17)
target_link_libraries(handbrake_node
  ${ackermann_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
install(TARGETS handbrake_node DESTINATION lib/${PROJECT_NAME})

# drift parking into a gap, the same way
add_executable(gap_node src/gap_node.cpp)
target_compile_features(gap_node PUBLIC cxx_std_17)
target_link_libraries(gap_node
  ${ackermann_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
install(TARGETS gap_node DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/friction_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
