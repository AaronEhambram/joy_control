cmake_minimum_required(VERSION 3.8)
project(joy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#### youbot driver dependencies
## check required prerequisites
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/external/cmake_modules")
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)
set(YouBot_Driver_DIR "/home/aaron/workspace/libraries/youbot_driver/build")
set(youbot_driver_PACKAGE_PATH "/home/aaron/workspace/libraries/youbot_driver/install")
set(OODL_YOUBOT_INCLUDE_DIR "/home/aaron/workspace/libraries/youbot_driver")
find_package(OODL_YouBot REQUIRED)
add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}/")
## Set include directories
INCLUDE_DIRECTORIES(
	${OODL_YOUBOT_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
  include
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

# create executable
add_executable(joy_control_node src/joy_control_node.cpp)

# linking libraries
target_link_libraries(joy_control_node 
	${OODL_YOUBOT_LIBRARIES}
	${Boost_LIBRARIES}
	pthread
)
set_target_properties(joy_control_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
ament_target_dependencies(joy_control_node rclcpp sensor_msgs)

# install the executables
install(TARGETS
  joy_control_node
  DESTINATION lib/${PROJECT_NAME})
#install launch-files
install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
