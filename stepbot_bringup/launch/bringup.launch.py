from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import xacro

desc_dir_path = join(get_package_share_directory('stepbot_description'))
xacro_path = join(desc_dir_path, 'urdf', 'stepbot.xacro')
urdf_path  = join(desc_dir_path, 'urdf', 'stepbot.urdf')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # load xacro
    doc = xacro.process_file(xacro_path)
    # xacro to urdf
    robot_desc = doc.toprettyxml(indent='  ')
    # write urdf to urdf_path
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    
    print("urdf_file_name : {}".format(urdf_path))

    rsp_params = {'robot_description' : robot_desc}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params, {'use_sim_time': use_sim_time}]
    )

    container = Node(
      name='stepbot_container',
      package='rclcpp_components',
      executable='component_container',
      output='both',
    )

    components = LoadComposableNodes(
      target_container='stepbot_container',
      composable_node_descriptions=[
        ComposableNode(
          package='stepbot_node',
          plugin='stepbot_status::StepbotStatus',
          name='stepbot_status',
          parameters=[join(get_package_share_directory("stepbot_bringup"), 'config', 'stepbot_status.yaml')],
        )
      ]
    )
        
    uros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial","--dev","/dev/ttyACM0"],
        output="screen"
    )

    ldlidar_pkg_dir = LaunchConfiguration(
        'ldlidar_pkg_dir',
        default=join(get_package_share_directory('ldlidar'), 'launch')
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ldlidar_pkg_dir, '/ldlidar.launch.py']),
            launch_arguments={
                'serial_port': '/dev/ttyAMA1',
                'lidar_frame': 'base_scan'
            }.items()
        ),

        robot_state_publisher_node,
        uros_agent_node,
        container,
        components
    ])

