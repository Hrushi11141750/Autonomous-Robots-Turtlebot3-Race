from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autorace_real',
            executable='enter_tunnel_real',
            name='enter_tunnel_real',
            output='screen'
        ),
        TimerAction(
            period=5.0,  # wait 5 seconds before calling service
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/detect_tunnel_sign', 'autorace_real_interfaces/srv/DetectTunnelSign'],
                    shell=True,
                    output='screen'
                )
            ]
        )
    ])
