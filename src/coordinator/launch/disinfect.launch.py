from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    replan_launch_arg = DeclareLaunchArgument(
        'replan', default_value=EnvironmentVariable('REPLAN')
    )
    bdi_launch_arg = DeclareLaunchArgument(
        'bdi', default_value=EnvironmentVariable('BDI')
    )

    return LaunchDescription([
        replan_launch_arg,
        bdi_launch_arg,
        Node(
            package='coordinator',
            executable='disinfect_coordinator',
            name='disinfect_coordinator',
            parameters=[{
                'replan': LaunchConfiguration('replan'),
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
    ])