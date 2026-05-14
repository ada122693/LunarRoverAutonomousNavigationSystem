from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #DeclareLaunchArgument(
        #    'rover_id',
        #    default_value='1',
        #    description='Rover ID for namespacing the action server'
        #),
        Node(
            package='haphazard',
            executable='path_schedule_plan_server',
            name='path_schedule_plan_server',
            #namespace=['rover_', LaunchConfiguration('rover_id')],
            output='screen',
            parameters=[{'use_sim_time': False}],
        )
    ])
