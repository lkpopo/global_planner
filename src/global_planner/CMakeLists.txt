cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(BUILD_SHARED_LIBS OFF)

# 指定交叉编译工具链
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# Eigen3
set(Eigen3_INCLUDE_DIR "/home/zxhc/cross/install/aarch64/include/eigen3")
set(YAMLCPP_LIB "/home/zxhc/cross/install/aarch64/lib/libyaml-cpp.a")
# PCL
set(PCL_ROOT "/home/zxhc/cross/install/aarch64/pcl")
set(PCL_INCLUDE_DIRS "${PCL_ROOT}/include/pcl-1.12")
set(PCL_LIBRARY_DIRS "${PCL_ROOT}/lib")
link_directories(${PCL_LIBRARY_DIRS})

# include dirs
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 库
add_library(occupy_map src/occupy_map.cpp)
target_link_libraries(occupy_map
    ${PCL_LIB_DIR}/libpcl_common.a
    ${PCL_LIB_DIR}/libpcl_io.a
    ${PCL_LIB_DIR}/libpcl_kdtree.a
    ${PCL_LIB_DIR}/libpcl_filters.a
)

add_library(A_star src/A_star.cpp)
target_link_libraries(A_star occupy_map)

add_library(global_planner src/global_planner.cpp)
target_link_libraries(global_planner
    A_star
    # ${PCL_LIB_DIR}/libpcl_common.a
    ${YAMLCPP_LIB}   # yaml-cpp 静态库路径
)
# add_custom_command(TARGET global_planner POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E echo "Merging static libs..."
#     COMMAND ${CMAKE_AR} x $<TARGET_FILE:global_planner> 
#     COMMAND ${CMAKE_AR} x ./libA_star.a
#     COMMAND ${CMAKE_AR} x ./liboccupy_map.a
#     COMMAND ${CMAKE_AR} x /home/zxhc/cross/install/aarch64/lib/libyaml-cpp.a
#     COMMAND ${CMAKE_AR} rcs $<TARGET_FILE:global_planner> *.o
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
# )