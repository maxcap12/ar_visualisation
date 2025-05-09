cmake_minimum_required(VERSION 3.8)
project(ar_visualisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(situational_graphs_msgs REQUIRED)
find_package(ar_visualisation_msgs REQUIRED)

add_executable(map_creator 
    src/map_creator.cpp 
    src/mesh_wall.cpp 
    src/map_marker.cpp
)

target_include_directories(map_creator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(map_creator PUBLIC c_std_99 cxx_std_17) 

ament_target_dependencies(map_creator
  rclcpp
  rclpy
  geometry_msgs
  situational_graphs_msgs
  ar_visualisation_msgs
  visualization_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ar_visualisation/server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS map_creator
  DESTINATION lib/${PROJECT_NAME}
)
  
install(
  DIRECTORY include/
  DESTINATION include
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

ament_export_include_directories(include)
ament_package()
