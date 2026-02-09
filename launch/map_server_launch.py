from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map Server Node (lifecycle node)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/opt/ros/humble/share/turtlebot4_navigation/maps/warehouse.yaml'
            }]
        ),

        # Lifecycle Manager Node (automatically activates the map server)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'node_names': ['map_server'],
                'autostart': True   # automatically transitions map_server to active
            }]
        )
    ])
