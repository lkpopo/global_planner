
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(   # 如果你有全局头文件的话
    /usr/include/eigen3                  # Eigen
    ${PCL_INCLUDE_DIRS}                  # PCL include
    # 如果 yaml-cpp 不在系统路径，手动加上
    /usr/include/yaml-cpp
)

# -------------------- Libraries --------------------
# PCL, yaml-cpp, pthread
set(GLOBAL_PLANNER_LIBS
    OpenMP::OpenMP_CXX   
    pthread
    ${PCL_LIBRARIES}
    yaml-cpp
)

