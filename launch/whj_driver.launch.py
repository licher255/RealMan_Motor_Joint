from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        DeclareLaunchArgument(
            'motor_ids',
            default_value='[1, 2, 3, 4, 5, 6]',
            description='List of motor IDs'
        ),
        DeclareLaunchArgument(
            'joint_names',
            default_value='["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]',
            description='List of joint names'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='50.0',
            description='State update rate in Hz'
        ),
        
        Node(
            package='realman_whj_driver',
            executable='whj_driver_node',
            name='whj_driver',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'motor_ids': LaunchConfiguration('motor_ids'),
                'joint_names': LaunchConfiguration('joint_names'),
                'update_rate': LaunchConfiguration('update_rate'),
            }],
            remappings=[
                ('joint_states', '/joint_states'),
                ('position_command', '/position_command'),
            ]
        )
    ])
