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
            executable='disinfect_environment',
            name=['disinfect_environment'],
            parameters=[{
                'problem_rate': LaunchConfiguration('problem_rate'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='disinfect_planner',
            name='disinfect_planner'
        ),
        Node(
            package='murosa_plan',
            executable='nurse_disinfect',
            name='nurse_disinfect1'
        ),
        Node(
            package='murosa_plan',
            executable='nurse_disinfect',
            name='nurse_disinfect2'
        ),
        Node(
            package='murosa_plan',
            executable='uvdrobot',
            name='uvdrobot1'
        ),
        Node(
            package='murosa_plan',
            executable='uvdrobot',
            name='uvdrobot2'
        ),
        Node(
            package='murosa_plan',
            executable='spotrobot',
            name='spotrobot1'
        ),
        Node(
            package='murosa_plan',
            executable='spotrobot',
            name='spotrobot2'
        ),
    ])