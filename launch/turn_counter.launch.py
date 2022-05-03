#! /usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    turn_counter_node = Node(
        package="tether_manager",
        executable="tether_turn_counter",
        name="tether_turn_counter"
    )
    
    return LaunchDescription([
        turn_counter_node
        ])