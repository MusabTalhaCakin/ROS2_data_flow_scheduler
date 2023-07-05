from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    num_clients_value = LaunchConfiguration('numClients', default='24')

    return LaunchDescription([
        DeclareLaunchArgument(
            'numClients',
            default_value=num_clients_value,
            description='Value for numClients'
        ),
        Node(
            package='centralized_data_flow_scheduler',
            executable='data_flow_scheduler',
            name='data_flow_scheduler',
            parameters=[
                {'numClients': num_clients_value}
            ]
        )
    ])
