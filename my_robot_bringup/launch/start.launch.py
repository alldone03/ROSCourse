from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("robot_news",'robot_news'),
    robot_news_station_c3po = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="robot_news_station_c3po",
        
        parameters=[
        {'robot_name':"c3po"},
    ]
    )
    robot_news_station_giskard = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="robot_news_station_giskard",
        
        parameters=[
        {'robot_name':"giskard"},
    ]
    )
    robot_news_station_bb8 = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="robot_news_station_bb8",
        
        parameters=[
        {'robot_name':"bb8"},
    ]
    )
    robot_news_station_daneel = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="robot_news_station_daneel",
        
        parameters=[
        {'robot_name':"daneel"},
    ]
    )
    robot_news_station_jander = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="robot_news_station_jander",
        
        parameters=[
        {'robot_name':"jander"},
    ]
    )

    smartphone = Node(
        package="my_cpp_pkg",
        executable="smartphone", 
        
    )

    ld.add_action(robot_news_station_c3po)
    ld.add_action(robot_news_station_giskard)
    ld.add_action(robot_news_station_bb8)
    ld.add_action(robot_news_station_daneel)
    ld.add_action(robot_news_station_jander)
    ld.add_action(smartphone)
    
    return ld
