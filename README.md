#### 仓库实现功能如下

##### 1. ROS2版本的基于A*的全局路径规划

规划出来的路径，在原有A*规划出来的路径上加了平滑处理，减少曲折。可以在planner_launch.py中设置相关参数，主要参数如下：

- `start_pos_x\y\z`设置起始点的位置
- `global_planner_ugv/pcd_path`设置pcd文件的路径
- `map/resolution`构建grid的分辨率
- `map/inflate`设置膨胀系数，（膨胀系数/分辨率=膨胀的倍数）

###### 1.1启动

```bash
ros2 launch global_planner planner_launch.py
```

###### 1.2设置目标点位置

另起一个终端，可以通过如下的命令，手动设置终点坐标

```bash
ros2 topic pub /global_planner/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -28.0, y: -25.0, z: 18.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

##### 2. ROS2版本的重定位

依赖于Sophus库，最新版本的sophus库依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

由粗到细的两阶段ICP的重定位，非常依赖初始位置值的设定，但是目前满足我的需求。定位融合了fastlio2的imu计算方式，将其整合到了一个节点当中。

###### 2.1启动

```bash
ros2 launch localizer localizer_launch.py
```

###### 2.2设定位置初值并检验结果

设置你当前位置的初值，以及需要加载的全局点云的位置

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```

设置完毕之后，你可以载着雷达四处跑，或者播放bag。中途你可以通过如下，命令检查当前的定位结果是否准确。

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

