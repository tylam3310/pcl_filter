cmake_minimum_required(VERSION 3.5)
project(pcl_filter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Add executable
add_executable(filter src/filter/filter_node.cpp src/filter/filter_component.cpp)
add_executable(clustering src/clustering/clustering_node.cpp src/clustering/clustering_component.cpp)

ament_target_dependencies(filter
  rclcpp 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(clustering
  rclcpp 
  sensor_msgs
  pcl_conversions
)

target_link_libraries(filter ${PCL_LIBRARIES})
target_link_libraries(clustering ${PCL_LIBRARIES})

# Install Cpp executables
install(TARGETS
  filter
  clustering
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()