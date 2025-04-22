from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    problem_rate_launch_arg = DeclareLaunchArgument(
        'problem_rate', default_value=EnvironmentVariable('PROBLEM_RATE')
    )

    return LaunchDescription([
        problem_rate_launch_arg,
        Node(
            package='murosa_plan',
            executable='environment',
            name=['environment'],
            parameters=[{
                'problem_rate': LaunchConfiguration('problem_rate'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='planner',
            name='planner'
        ),
        Node(
            package='murosa_plan',
            executable='patrol',
            name='patrol'
        ),
    ])