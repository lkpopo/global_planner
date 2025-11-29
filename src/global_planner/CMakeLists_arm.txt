cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(CMAKE_CXX_STANDARD 17)
set(BUILD_SHARED_LIBS OFF)

# ============================================================
# 交叉编译库搜索路径
# ============================================================
set(AARCH64_SYSROOT "/home/zxhc/cross/install/aarch64")

# 头文件
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${AARCH64_SYSROOT}/include
    ${AARCH64_SYSROOT}/include/eigen3
)

# 库目录
link_directories(
    ${AARCH64_SYSROOT}/lib
)

# ============================================================
# 手动指定 PCL Config 路径（关键！）
# ============================================================
set(PCL_DIR "${AARCH64_SYSROOT}/share/pcl-1.12")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
/home/zxhc/cross/install/aarch64/include
)
add_definitions(${PCL_DEFINITIONS})

# ============================================================
# yaml-cpp 路径
# ============================================================
# set(yaml-cpp_DIR "${AARCH64_SYSROOT}/lib/cmake/yaml-cpp")

# find_package(yaml-cpp REQUIRED)

# ============================================================
# 源文件
# ============================================================
set(SOURCES
    src/global_planner.cpp
    src/A_star.cpp
    src/occupy_map.cpp
)

add_library(global_planner STATIC ${SOURCES})

# ============================================================
# 链接库
# ============================================================
target_link_libraries(global_planner
    pthread
    /home/zxhc/cross/install/aarch64/lib/libyaml-cpp.a
    ${PCL_LIBRARIES}
)
