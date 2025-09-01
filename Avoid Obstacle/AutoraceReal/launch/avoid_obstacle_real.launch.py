from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch the avoid_obstacle_real_action node
        Node(
            package='autorace_real',
            executable='avoid_obstacle_real_action',
            name='avoid_obstacle_real_action',
            output='screen'
        ),

        # Wait 5 seconds, then send an empty goal to the action server
        TimerAction(
            period=5.0,  # Adjust as needed
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'action', 'send_goal',
                        '/avoid_obstacle_real_action',
                        'autorace_real_interfaces/action/AvoidObstacle',
                        '{}'
                    ],
                    shell=True,
                    output='screen'
                )
            ]
        )
    ])
