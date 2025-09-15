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
            {"map/resolution": 0.2},
            {
                "global_planner/pcd_path": "/home/dpf/Workspace/global_planner/building.pcd"
            },
            {"map/inflate": 0.5},
            {"start_pos_x": 0.0},
            {"start_pos_y": 0.0},
            {"start_pos_z": 8.0},
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
