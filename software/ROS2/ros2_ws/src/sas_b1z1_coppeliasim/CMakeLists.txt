cmake_minimum_required(VERSION 3.8)
project(sas_b1z1_coppeliasim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/sas_b1z1_coppeliasim_ros.cpp
    src/RobotDriverCoppeliaSimUnitreeB1Z1.cpp
    )
target_link_libraries(${PROJECT_NAME}
    dqrobotics-interface-coppeliasim-zmq
)
ament_target_dependencies(${PROJECT_NAME}
rclcpp geometry_msgs std_msgs sensor_msgs sas_common sas_core sas_conversions Eigen3 tf2 tf2_ros
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${REPOSITORY_DIR}/software/include/
    $<BUILD_INTERFACE:${REPOSITORY_DIR}/software/src/
    )


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
Eigen3 rclcpp geometry_msgs std_msgs sensor_msgs sas_common sas_core sas_conversions
    )

install(
    DIRECTORY include/
    DESTINATION include)

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

# CPP Binary Block [BEGIN] #

# vvvvvvvvvvvvvvvvvvvvvvvv #

# https://ros2-tutorial.readthedocs.io/en/latest/

# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block

# we use set--unset

set(RCLCPP_LOCAL_BINARY_NAME sas_b1z1_coppeliasim_node)
add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    src/sas_b1z1_coppeliasim_node.cpp
    )

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
    rclcpp
    sas_common
    sas_core
    #sas_robot_driver
    )

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
    ${PROJECT_NAME}
    )

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_20)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
    DESTINATION lib/${PROJECT_NAME})
unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
