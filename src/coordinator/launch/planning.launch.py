from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    replan_launch_arg = DeclareLaunchArgument(
        'replan', default_value=EnvironmentVariable('REPLAN')
    )

    return LaunchDescription([
        replan_launch_arg,
        Node(
            package='coordinator',
            executable='coordinator',
            name='coordinator',
            parameters=[{
                'replan': LaunchConfiguration('replan'),
            }]
        ),
    ])