cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# -------------------- 外部依赖 --------------------

find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# include dirs
include_directories(
    include
    ${PROJECT_SOURCE_DIR}/include
    /usr/include/eigen3
    ${PCL_INCLUDE_DIRS}

)

set(SOURCES
    src/planner.cpp
    src/A_star.cpp
    src/occupy_map.cpp
)

#---------自己调试的时候用这个----------------
include(${CMAKE_CURRENT_SOURCE_DIR}/3party/miniz/miniz.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/3party/pugixml/pugixml.cmake)
add_executable(main src/main.cpp src/WaylineManager.cpp)

add_library(planner STATIC ${SOURCES})

target_link_libraries(
    main
    planner
    OpenMP::OpenMP_CXX
    pthread
    miniz_lib 
    pugixml_lib
    ${PCL_LIBRARIES}
    yaml-cpp
)
#---------自己调试的时候用这个----------------

# ------------------- 仅库文件 -------------------
# add_library(planner STATIC ${SOURCES})

# target_link_libraries(
#     planner
#     OpenMP::OpenMP_CXX
#     pthread
#     ${PCL_LIBRARIES}
#     yaml-cpp
# )
# ------------------- 仅库文件 -------------------