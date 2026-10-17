cmake_minimum_required(VERSION 3.22)
project(sas_robot_driver_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)
# https://github.com/juanjqo/cpp-interface-mujoco (not a ROS package: install it first). If it is not
# installed in a system path, add its prefix to CMAKE_PREFIX_PATH.
find_package(dqrobotics-interface-mujoco 1.1 REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.
add_library(${PROJECT_NAME} SHARED
    src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    sas_core
    sas_common
    Eigen3
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC dqrobotics
    # Private: the interface is hidden behind a pointer in the header
    PRIVATE dqrobotics_extensions::interface-mujoco
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    sas_core
    sas_common
    Eigen3
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME sas_robot_driver_mujoco_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    src/${RCLCPP_LOCAL_BINARY_NAME}.cpp
)

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
    rclcpp
    sas_common
    sas_core
    sas_robot_driver
)

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
    ${PROJECT_NAME}
    dqrobotics
)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
    DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

################################
# Launch/Config Block [BEGIN]  #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvv #
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/
)
# ^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# Launch/Config Block [END]  #
##############################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
