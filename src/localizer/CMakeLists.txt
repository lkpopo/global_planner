cmake_minimum_required(VERSION 3.10)
project(localizer)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

include_directories(/home/dji/Test/party/include
/home/dji/Test/global_planner/src/localizer/src
/home/dji/Test/global_planner/src/localizer/src/localizerss
)

# ===== 找第三方库 =====
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

# ===== 源文件 =====
set(SRC_LIST 
    src/localizer.cpp
    src/localizers/commons.cpp
    src/localizers/icp_localizer.cpp
    src/localizers/ikd_Tree.cpp
    src/localizers/lidar_processor.cpp
    src/localizers/imu_processor.cpp
    src/localizers/ieskf.cpp
)

# ===== 生成静态库 .a =====
add_library(localizer STATIC ${SRC_LIST})

target_link_libraries(localizer
    ${PCL_LIBRARIES}
    yaml-cpp
)
