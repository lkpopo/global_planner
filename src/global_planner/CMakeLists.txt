cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(BUILD_SHARED_LIBS OFF)


# -------------------- 外部依赖 --------------------
# Eigen3
set(Eigen3_INCLUDE_DIR "/usr/include/eigen3")

# YAML
set(YAMLCPP_LIB "/usr/lib/aarch64-linux-gnu/libyaml-cpp.a")

# 查找PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
)
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
    pthread
    ${PCL_LIBRARIES}
    yaml-cpp
)
