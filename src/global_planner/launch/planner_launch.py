#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("global_planner"), "rviz", "global_planner.rviz"]
    )
    
    # 节点配置
    global_planner_node = Node(
        package="global_planner",
        executable="global_planner_node",
        name=["global_planner"],
        output="screen",
        parameters=[
            {"map/origin_x": -20.0},
            {"map/origin_y": -20.0},
            {"map/origin_z": 0.0},
            {"map/map_size_x": 100.0},
            {"map/map_size_y": 100.0},
            {"map/map_size_z": 20.0},
            {"map/queue_size": 1},
            {"map/border": True},
            {"map/resolution": 0.3},
            {
                "global_planner/pcd_path": "/home/dpf/Workspace/global_planner/test.pcd"
            },
            {"map/inflate": 0.3},
            {"global_planner/cost_inflate": 2},
            {"astar/lambda_cost": 10.0},
            {"start_pos_x": 2.7},
            {"start_pos_y": 2.0},
            {"start_pos_z": 3.0},
        ],
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_cfg],
        )


    return LaunchDescription([global_planner_node,rviz2])
