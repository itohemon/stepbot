import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

#desc_dir_path = os.path.join(get_package_share_directory('stepbot_description'))
#xacro_path = os.path.join(desc_dir_path, 'urdf', 'stepbot.xacro')
#urdf_path  = os.path.join(desc_dir_path, 'urdf', 'stepbot.urdf')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ## load xacro
    #doc = xacro.process_file(xacro_path)
    ## xacro to urdf
    #robot_desc = doc.toprettyxml(indent='  ')
    ## write urdf to urdf_path
    #f = open(urdf_path, 'w')
    #f.write(robot_desc)
    #f.close()
    
    #print("urdf_file_name : {}".format(urdf_path))

    #rsp_params = {'robot_description' : robot_desc}

    #robot_state_publisher_node = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    output='screen',
    #    parameters=[rsp_params, {'use_sim_time': use_sim_time}]
    #)

    #status_node = Node(
    #    package='stepbot_node',
    #    executable='stepbot_status',
    #    parameters=[os.path.join(get_package_share_directory("stepbot_node"), 'config', 'stepbot_status.yaml')],
    #    output='screen'
    #)
        
    uros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial","--dev","/dev/ttyACM0"],
        output="screen"
    )

    #ldlidar_pkg_dir = LaunchConfiguration(
    #    'ldlidar_pkg_dir',
    #    default=os.path.join(get_package_share_directory('ldlidar'), 'launch')
    #)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([ldlidar_pkg_dir, '/ldlidar.launch.py']),
        #    launch_arguments={
        #        'serial_port': '/dev/ttyAMA1',
        #        'lidar_frame': 'base_scan'
        #    }.items()
        #),

        #robot_state_publisher_node,
        uros_agent_node,
        #status_node
    ])

