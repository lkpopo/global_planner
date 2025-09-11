#!/usr/bin/env python3
import os

from sympy import false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ugv_id = 1

    # 节点配置
    global_planner_node = Node(
        package="global_planner_ugv",
        executable="global_planner_ugv_node",
        name=["global_planner_ugv_main_1"],
        output="screen",
        parameters=[
            {"global_planner_ugv/ugv_id": 1},
            {"global_planner_ugv/ugv_height": 0.0},
            {"global_planner_ugv/replan_time": 0.2},
            {"global_planner_ugv/track_frequency": 0.1},
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
                "global_planner_ugv/pcd_path": "/home/dpf/Workspace/uav_global_planner/test1.pcd"
            },
            {"map/inflate": 0.3},
            {"global_planner_ugv/odom_inflate": 0.3},
            {"global_planner_ugv/cost_inflate": 2},
            {"astar/lambda_cost": 10.0},
            {"start_pos_x": 2.7},
            {"start_pos_y": -4.0},
            {"start_pos_z": 5.0},
        ],
        remappings=[
            (
                "/ugv1/prometheus/global_planner_ugv/global_pcl",
                "/map_generator/global_cloud",
            ),
        ],
    )

    return LaunchDescription([global_planner_node])
