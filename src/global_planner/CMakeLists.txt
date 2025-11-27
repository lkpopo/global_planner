cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(BUILD_SHARED_LIBS OFF)

# -------------------- 交叉编译工具链 --------------------
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER   /usr/bin/aarch64-linux-gnu-gcc-9)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++-9)

# -------------------- 外部依赖 --------------------
# Eigen3
set(Eigen3_INCLUDE_DIR "/home/zxhc/cross/install/aarch64/include/eigen3")

# YAML
set(YAMLCPP_LIB "/home/zxhc/cross/install/aarch64/lib/libyaml-cpp.a")

# 查找PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)

# include dirs
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${BOOST_ROOT}/include
)

# ------------------- 全功能静态库 -------------------
set(SOURCES
    src/global_planner.cpp
    src/A_star.cpp
    src/occupy_map.cpp
)

add_library(global_planner STATIC ${SOURCES})
target_link_libraries(global_planner 
PUBLIC
    ${PCL_LIBRARIES}
    yaml-cpp
)
