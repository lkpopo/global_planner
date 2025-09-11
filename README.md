ROS2版本的基于A*的全局路径规划，在planner_launch.py中设置相关参数。[Prometheus](https://github.com/amov-lab/Prometheus)仓库摘录下来的，并做了ROS2的适配，原来是ROS1版本的。目前运行起来没问题，当然还有很多冗余的代码没有完全删除，结构还可以优化。

- `start_pos_x\y\z`设置起始点的位置
- `global_planner_ugv/pcd_path`设置pcd文件的路径
- `map/map_size_x\y\z`设置地图的范围大小
- `map/origin_x\y\z`设置点云的起始位置，也就是从哪个点开始铺上点云
- `map/resolution`构建grid的分辨率

通过如下命令可以手动的设定终点的坐标，然后打开rviz2订阅相关内容即可查看。

```bash
ros2 topic pub /ugv1/prometheus/global_planner_ugv/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 12.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

