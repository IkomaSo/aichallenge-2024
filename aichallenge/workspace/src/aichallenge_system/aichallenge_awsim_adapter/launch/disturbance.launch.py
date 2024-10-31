from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aichallenge_awsim_adapter',
            executable='make_disturbance',
            name='make_disturbance',
            prefix='xterm -e',
        )
    ])