from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
    problem_rate_launch_arg = DeclareLaunchArgument(
        'problem_rate', default_value=EnvironmentVariable('PROBLEM_RATE')
    )
    bdi_launch_arg = DeclareLaunchArgument(
        'bdi', default_value=EnvironmentVariable('BDI')
    )

    return LaunchDescription([
        problem_rate_launch_arg,
        bdi_launch_arg,
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
            executable='navigator',
            name='navigator'
        ),
        Node(
            package='murosa_plan',
            executable='nurse',
            name='nurse1',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='nurse',
            name='nurse2',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='nurse',
            name='nurse3',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='nurse',
            name='nurse4',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='uvd',
            name='uvd1',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='uvd',
            name='uvd2',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='spot',
            name='spot1',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='spot',
            name='spot2',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='collector',
            name='collector1',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='collector',
            name='collector2',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
        Node(
            package='murosa_plan',
            executable='arm',
            name='arm1',
            parameters=[{
                'bdi': LaunchConfiguration('bdi'),
            }]
        ),
    ])