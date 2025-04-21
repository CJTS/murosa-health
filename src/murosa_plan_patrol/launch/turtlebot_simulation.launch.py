from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    replan_launch_arg = DeclareLaunchArgument(
        'replan', default_value=EnvironmentVariable('REPLAN')
    )
    problem_rate_launch_arg = DeclareLaunchArgument(
        'problem_rate', default_value=EnvironmentVariable('PROBLEM_RATE')
    )

    # Include TurtleBot3 Gazebo launch file
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    # Include your existing nodes
    ld = LaunchDescription([
        replan_launch_arg,
        problem_rate_launch_arg,
        turtlebot3_gazebo,
        Node(
            package='murosa_plan_patrol',
            executable='coordinator',
            name='coordinator',
            parameters=[{
                'replan': LaunchConfiguration('replan'),
            }]
        ),
        Node(
            package='murosa_plan_patrol',
            executable='environment',
            name=['environment'],
            parameters=[{
                'problem_rate': LaunchConfiguration('problem_rate'),
            }]
        ),
        Node(
            package='murosa_plan_patrol',
            executable='planner',
            name='planner'
        ),
        Node(
            package='murosa_plan_patrol',
            executable='robot',
            name='robot'
        ),
    ])

    return ld 