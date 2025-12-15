from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS-TCP Endpoint (for Unity connection)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            parameters=[{
                'ROS_IP': '0.0.0.0',
                'ROS_TCP_PORT': 10000
            }]
        ),

        # Path Planner for Robot 1
        Node(
            package='path_planner',
            executable='path_planner_server',
            name='path_planner_server_robot1',
            parameters=[{
                'robot_name': 'robot1'
            }]
        ),

        # Path Planner for Robot 2
        Node(
            package='path_planner',
            executable='path_planner_server',
            name='path_planner_server_robot2',
            parameters=[{
                'robot_name': 'robot2'
            }]
        ),

        # Path Planner for Robot 3
        Node(
            package='path_planner',
            executable='path_planner_server',
            name='path_planner_server_robot3',
            parameters=[{
                'robot_name': 'robot3'
            }]
        ),

        # Path Planner for Robot 4
        Node(
            package='path_planner',
            executable='path_planner_server',
            name='path_planner_server_robot4',
            parameters=[{
                'robot_name': 'robot4'
            }]
        ),

        # Monitor Node
        Node(
            package='monitor_node',
            executable='monitor_node',
            name='monitor_node',
            parameters=[
                {'robot_names': ['robot1', 'robot2', 'robot3', 'robot4']},
                {'spawn_interval': 5.0},
                {'min_packages_per_shelf': 2},
                {'max_packages_per_shelf': 5}
            ]
        ),
        
        # Robot 1
        Node(
            package='robot_nodes',
            executable='robot_node',
            name='robot1',
            parameters=[{
                'robot_name': 'robot1',
                'all_robot_names': ['robot1', 'robot2', 'robot3', 'robot4']
            }]
        ),
        
        # Robot 2
        Node(
            package='robot_nodes',
            executable='robot_node',
            name='robot2',
            parameters=[{
                'robot_name': 'robot2',
                'all_robot_names': ['robot1', 'robot2', 'robot3', 'robot4']
            }]
        ),

        # Robot 3
        Node(
            package='robot_nodes',
            executable='robot_node',
            name='robot3',
            parameters=[{
                'robot_name': 'robot3',
                'all_robot_names': ['robot1', 'robot2', 'robot3', 'robot4']
            }]
        ),

        # Robot 4
        Node(
            package='robot_nodes',
            executable='robot_node',
            name='robot4',
            parameters=[{
                'robot_name': 'robot4',
                'all_robot_names': ['robot1', 'robot2', 'robot3', 'robot4']
            }]
        ),
    ])
