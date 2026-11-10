from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Driver del Tello
        Node(
            package='tello_driver',
            executable='tello_driver_node',
            name='tello_driver',
            output='screen'
        ),
        
        # 2. Nodo de Visión
        Node(
            package='tello_driver',
            executable='tello_vision_node',
            name='tello_vision',
            output='screen'
        ),
        
        # 3. Nodo de Control
        Node(
            package='tello_driver',
            executable='tello_control_node',
            name='tello_control',
            output='screen'
        ),
    ])
