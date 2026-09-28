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

# the solver in C from ocp.py, with the parameters of vehicle_params.yaml built in
set(PARAMS ${ackermann_description_DIR}/../config/vehicle_params.yaml)
set(SOLVER_DIR ${CMAKE_CURRENT_BINARY_DIR}/solver)
execute_process(
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate.py ${PARAMS} ${SOLVER_DIR}
  RESULT_VARIABLE generate_result)
if(generate_result)
  message(FATAL_ERROR "scripts/generate.py failed")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${PARAMS}
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate.py
  ${CMAKE_CURRENT_SOURCE_DIR}/ackermann_nmpc/model.py
  ${CMAKE_CURRENT_SOURCE_DIR}/ackermann_nmpc/ocp.py)

add_executable(nmpc_node src/nmpc_node.cpp)
target_compile_features(nmpc_node PUBLIC cxx_std_17)
target_include_directories(nmpc_node PRIVATE
  ${SOLVER_DIR}
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include)
target_link_directories(nmpc_node PRIVATE ${SOLVER_DIR} ${ACADOS_DIR}/lib)
target_link_libraries(nmpc_node
  acados_ocp_solver_nmpc acados hpipm blasfeo
  ${ackermann_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
# RPATH, not RUNPATH: libacados.so finds hpipm and blasfeo through it too
set_target_properties(nmpc_node PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${ACADOS_DIR}/lib")
target_link_options(nmpc_node PRIVATE -Wl,--disable-new-dtags)

install(TARGETS nmpc_node DESTINATION lib/${PROJECT_NAME})
install(FILES ${SOLVER_DIR}/libacados_ocp_solver_nmpc.so DESTINATION lib)
ament_python_install_package(${PROJECT_NAME})

ament_package()
