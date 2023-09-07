cmake_minimum_required(VERSION 3.8)
project(riptide_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(riptide_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


# # orientation_controller
# add_executable(orientation_controller src/orientation_controller.cpp)
# target_include_directories(orientation_controller PRIVATE include)
# target_link_libraries(orientation_controller Eigen3::Eigen)
# ament_target_dependencies(orientation_controller rclcpp sensor_msgs riptide_msgs std_srvs visualization_msgs geometry_msgs std_msgs)

# orthogonal_controller
add_executable(orthogonal_controller src/orthogonal_controller.cpp)
target_include_directories(orthogonal_controller PRIVATE include)
target_link_libraries(orthogonal_controller Eigen3::Eigen)
ament_target_dependencies(orthogonal_controller rclcpp sensor_msgs riptide_msgs std_srvs visualization_msgs geometry_msgs std_msgs)

# # state_machine
# add_executable(state_machine src/state_machine.cpp)
# target_include_directories(state_machine PRIVATE include)
# target_link_libraries(state_machine Eigen3::Eigen)
# ament_target_dependencies(state_machine rclcpp riptide_msgs sensor_msgs)

# stable cycles
add_executable(stable_cycles src/stable_cycles.cpp)
target_include_directories(stable_cycles PRIVATE include)
target_link_libraries(stable_cycles Eigen3::Eigen)
ament_target_dependencies(stable_cycles rclcpp riptide_msgs sensor_msgs)

# Install
install(TARGETS orthogonal_controller stable_cycles DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Install Python executables
install(PROGRAMS
  scripts/001-1s_1m.py
  scripts/002-dolphin.py
  scripts/003-2m_1m_2m.py
  scripts/004-2m_fc.py
  # scripts/004-2m_fc_sim.py
  scripts/005-angular_velocity.py
  scripts/006-cycles.py
  scripts/007-R.py
  scripts/008-cap.py
  scripts/010-immersion.py
  scripts/011-immersion_cap.py
  DESTINATION lib/${PROJECT_NAME}
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
