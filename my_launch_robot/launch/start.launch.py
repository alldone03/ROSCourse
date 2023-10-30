from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    turtle_controller = Node(
            package="turtlesim_catch_them_all",
            executable="turtlesim_controller",      
            parameters=[{"target_x":11.0},{'target_y':0.0}]
    )
    
    ld.add_action(turtlesim)
    ld.add_action(turtle_controller)
    return ld
