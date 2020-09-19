from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # launch paths
    metrobot_description_dir = get_package_share_directory('metrobot_description')
    metrobot_simulation_dir = get_package_share_directory('metrobot_simulation')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    launch_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py'])
    )

    set_use_sim_time_cmd = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen')

    launch_robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([metrobot_description_dir, '/launch/metrobot_description.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([metrobot_simulation_dir, '/launch/spawn_metrobot.launch.py'])
    )

    ld = LaunchDescription()
    ld.add_action(launch_gazebo_cmd)
    ld.add_action(set_use_sim_time_cmd)
    ld.add_action(launch_robot_description_cmd)
    ld.add_action(spawn_robot_cmd)
    return ld