cmake_minimum_required(VERSION 3.5)
project(global_planner)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../common/include
)

# 库
add_library(occupy_map src/occupy_map.cpp)
target_link_libraries(occupy_map ${PCL_LIBRARIES})
ament_target_dependencies(occupy_map
  rclcpp
  pcl_conversions
  sensor_msgs
  nav_msgs
  visualization_msgs
  laser_geometry
  tf2_ros
)

add_library(A_star src/A_star.cpp)
target_link_libraries(A_star occupy_map)

add_library(global_planner src/global_planner.cpp)
target_link_libraries(global_planner A_star ${PCL_LIBRARIES})
ament_target_dependencies(global_planner
    angles
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
)

# 节点
add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node global_planner ${PCL_LIBRARIES})
ament_target_dependencies(global_planner_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    pcl_conversions
)

# 安装节点可执行文件
install(TARGETS 
  global_planner
  global_planner_node
  occupy_map
  A_star
  EXPORT export_global_planner_lib
 DESTINATION lib/${PROJECT_NAME} # 对库来说可以省
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(include)
ament_export_targets(export_global_planner_lib HAS_LIBRARY_TARGET)
ament_package()
