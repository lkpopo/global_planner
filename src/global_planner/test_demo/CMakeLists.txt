cmake_minimum_required(VERSION 3.10)
project(test_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add third-party libraries
include(${CMAKE_CURRENT_SOURCE_DIR}/3party/miniz/miniz.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/3party/pugixml/pugixml.cmake)
include_directories(/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/include
/usr/include/eigen3
${PCL_INCLUDE_DIRS})

# Add executable
add_executable(test_demo main.cpp WaylineManager.cpp)

# Link against third-party libraries
target_link_libraries(test_demo PRIVATE miniz_lib pugixml_lib ${PCL_LIBRARIES})

# Include directories
target_include_directories(test_demo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
