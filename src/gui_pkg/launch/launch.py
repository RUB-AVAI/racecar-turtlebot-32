#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera_topic = Node(
        package="camera_pkg",
        executable="camera_topic"
    )
    camera = Node(
        package="camera_pkg",
        executable="camera_node"
    )
    cone_detection = Node(
        package="cone_detection_pkg",
        executable="cone_detection_node"
    )
    occupancy_map = Node(
        package="occupancy_map_pkg",
        executable="occupancy_node"
    )
    controller = Node(
        package="controller",
        executable="controller"
    )

    ld.add_action(camera_topic)
    ld.add_action(camera)
    ld.add_action(cone_detection)
    ld.add_action(occupancy_map)
    ld.add_action(controller)
    return ld