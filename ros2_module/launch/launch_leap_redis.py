from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Run in simulation mode'
    )
    sim = LaunchConfiguration('sim')
    return LaunchDescription([
        sim_arg,
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0},
                {'sim': sim},
            ]
        ),
        Node(
            package='leap_hand',
            executable='leap_hand_redis.py',
            name='leap_hand_redis',
            emulate_tty=True,
            output='screen'
        )
    ])
