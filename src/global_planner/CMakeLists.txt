cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# -------------------- 外部依赖 --------------------
# Eigen3
set(Eigen3_INCLUDE_DIR "/usr/include/eigen3")

# YAML
# set(YAMLCPP_LIB "/usr/lib/aarch64-linux-gnu/libyaml-cpp.a")

# 查找PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)

find_package(OpenMP REQUIRED)

# include dirs
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}

)
include(${CMAKE_CURRENT_SOURCE_DIR}/3party/miniz/miniz.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/3party/pugixml/pugixml.cmake)
# ------------------- 全功能静态库 -------------------
set(SOURCES
    src/planner.cpp
    src/A_star.cpp
    src/occupy_map.cpp
)
add_executable(main src/main.cpp WaylineManager.cpp)

add_library(global_planner STATIC ${SOURCES})

target_link_libraries(
    main
    global_planner
    OpenMP::OpenMP_CXX
    pthread
    miniz_lib 
    pugixml_lib
    ${PCL_LIBRARIES}
    yaml-cpp
)