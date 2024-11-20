from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    replan_launch_arg = DeclareLaunchArgument(
        'replan', default_value=EnvironmentVariable('REPLAN')
    )
    problem_rate_launch_arg = DeclareLaunchArgument(
        'problem_rate', default_value=EnvironmentVariable('PROBLEM_RATE')
    )

    ld = LaunchDescription([
        replan_launch_arg,
        problem_rate_launch_arg,
        node1 := Node(
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
        RegisterEventHandler(
            OnProcessExit(
                target_action=node1,
                on_exit=[LogInfo(msg='Node 1 has exited')]
            )
        ),
    ])

    return ld