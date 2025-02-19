cmake_minimum_required(VERSION 3.8)
project(depthimage_to_pointcloud2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(depthimage_to_pointcloud2_component SHARED
  src/depthimage_to_pointcloud2_component.cpp)

rclcpp_components_register_nodes(
  depthimage_to_pointcloud2_component 
  "depthimage_to_pointcloud2::Depthimage2Pointcloud2"
)


ament_auto_add_executable(depthimage_to_pointcloud2_node
  src/depthimage_to_pointcloud2_node.cpp
)

target_link_libraries(depthimage_to_pointcloud2_node
  depthimage_to_pointcloud2_component
)
 
install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES DESTINATION share/${PROJECT_NAME})
ament_auto_package()

