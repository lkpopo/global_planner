ROS2版本的基于A*的全局路径规划，在planner_launch.py中设置相关参数。[Prometheus](https://github.com/amov-lab/Prometheus)仓库摘录下来的，并做了ROS2的适配，原来是ROS1版本的。优化了规划后的路径，让其路径的转折点更少一些。

- `start_pos_x\y\z`设置起始点的位置
- `global_planner_ugv/pcd_path`设置pcd文件的路径
- `map/resolution`构建grid的分辨率
- `map/inflate`设置膨胀系数，（膨胀系数/分辨率=膨胀的倍数）

通过如下命令可以手动的设定终点的坐标，然后打开rviz2订阅相关内容即可查看。

```bash
ros2 topic pub /global_planner/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -28.0, y: -25.0, z: 18.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

