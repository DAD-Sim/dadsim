import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        SetParameter(name="simulation_step", value=0.1),
        Node(
            package='dadsim_timed',
            executable='timed',
            name='timed',
            parameters=[{
                'simulation_rate': 1.0,
                'disable_waiting_msg': True,
            }],
        ),
        SetParameter(name="use_sim_time", value=True),
        Node(
            package='dadsim_scheduler',
            executable='scheduler',
            name='scheduler'
        ),
        Node(
            package='dadsim_mapd_odr',
            executable='mapd',
            name='mapd',
            parameters=[{
                'odr_map_path': PathJoinSubstitution([FindPackageShare("dadsim_mapd_odr"), "maps", "sparrow.xodr"])
            }]
        ),
        Node(
            package='dadsim_object_manager',
            executable='object_manager',
            name='object_manager',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='dadsim_traffic_manager',
            executable='traffic_provider',
            name='traffic_provider',
            parameters=[{
                'config_file': PathJoinSubstitution([FindPackageShare("dadsim"), "config", "config.yaml"])
            }],
        ),
        Node(
            package='dadsim_traffic_manager',
            executable='traffic_manager',
            name='traffic_manager',
            parameters=[{
                'target_frame': 'manual/0',
                'target_coord': [0.0, 0.0],
                'target_range': 50.0,
                'expand_range': 20.0,
                'priority': 1,
            }],
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='dadsim',
            executable='manual',
            name='manual',
            parameters=[],
            emulate_tty=True,
            output='screen',
            arguments= [
                '126',
                '8.0',
                '-1.5',
            ]
        )
    ])

if __name__ == '__main__':
    launch_description = generate_launch_description()
    launch.launch(launch_description)
