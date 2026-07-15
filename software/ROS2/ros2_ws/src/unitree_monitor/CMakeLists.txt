cmake_minimum_required(VERSION 3.19)
project(unitree_monitor LANGUAGES CXX)

find_package(Qt6 6.4 REQUIRED COMPONENTS Core Widgets)
#sudo apt update
#sudo apt install qt6-base-dev qt6-tools-dev-tools

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_datalogger REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_robot_driver REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

qt_standard_project_setup()

set(REPOSITORY_DIR ${CMAKE_SOURCE_DIR}/../../../../../)
include_directories(${REPOSITORY_DIR}/submodules/unitree_kinematics/include/)
include_directories(${REPOSITORY_DIR}/software/submodules/sas_unitree_b1z1_control_template/utils)


include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    # Add other ROS2 package includes as needed
)

set(RCLCPP_LOCAL_BINARY_NAME unitree_monitor)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    #WIN32 MACOSX_BUNDLE
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mainwindow.ui
    ${REPOSITORY_DIR}/software/submodules/sas_unitree_b1z1_control_template/utils/sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.cpp
)

ament_target_dependencies(${RCLCPP_LOCAL_BINARY_NAME}
    rclcpp geometry_msgs std_msgs sensor_msgs sas_common sas_core sas_conversions sas_datalogger Eigen3 sas_robot_driver
)

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}

        Qt::Core
        Qt::Widgets
        robot_constraint_manager
)

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)


# Install the executable
install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



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

ament_package()

