#!/usr/bin/env python3
"""
ROS2 Launch file for complete evacuation system
Launches pathfinding node and Arduino bridge together
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for evacuation system"""
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='',
        description='Arduino serial port (empty for auto-detect)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Serial baudrate'
    )
    
    cell_size_arg = DeclareLaunchArgument(
        'cell_size',
        default_value='0.5',
        description='Maze cell size in meters'
    )
    
    # Pathfinding node
    pathfinding_node = Node(
        package='evacuation_system',
        executable='pathfinding_node',
        name='evacuation_pathfinding',
        output='screen',
        parameters=[{
            'cell_size': LaunchConfiguration('cell_size')
        }]
    )
    
    # Arduino bridge node
    arduino_bridge_node = Node(
        package='evacuation_system',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'cell_size': LaunchConfiguration('cell_size')
        }]
    )
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '$(find evacuation_system)/config/evacuation.rviz'],
        condition=None  # Add condition to enable/disable
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        cell_size_arg,
        pathfinding_node,
        arduino_bridge_node,
        # rviz_node,  # Uncomment to launch RViz automatically
    ])

