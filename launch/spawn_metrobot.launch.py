from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value='0',
        description='X location for robot to spawn'
    )

    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value='0',
        description='Y location for robot to spawn'
    )

    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value='0.3',
        description='Z location for robot to spawn'
    )

    spawn_br_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='model_spawner',
        arguments=['-entity', 'br', '-topic', '/robot_description', '-x', x, '-y', y, '-z', z]
    )

    ld = LaunchDescription()
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(spawn_br_cmd)
    return ld