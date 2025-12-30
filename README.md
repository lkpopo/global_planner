#### 仓库实现功能如下

1.请注意这个分支，不是基于ros的，编译完成之后就是一个.a的静态库。同时localizer里面的代码尽管修改了不需要ros，但是一直没有验证使用，可能会有错误。

2.编译完成后的libplanner.a库，主要有几个接口在planner.h里面有写到。下面着重介绍几个接口

- `setMap`设置pcd文件的路径，用于做路径规划
- `setConfig`设置配置文件路径，里面主要是一些A*和对点云进行网格化的一些配置，不用也可以，代码里指定了默认的参数
- `setOffset`通过此接口设置无人机当前相对于原点位置的偏移
- `setCurrLocation`通过此接口设置无人机的gps坐标
- `setWaypoint`设置无人机需要到达哪些航点
- `setPlannedWaypointsCallback`通过这个接口设置callback，路径规划完成之后 会返回一系列的路径点
- `setRealTimeUTMCallback`设置返回实时UTM坐标系下位置的callback（请注意坐标系，这里的utm坐标系是x指向正东，y指向正北）
- `setLogCallback`设置日志回调函数
- `setTaskStatusCallback`设置航线规划