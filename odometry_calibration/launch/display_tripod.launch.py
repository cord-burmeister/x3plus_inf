import os

from ament_index_python.packages import (
    get_package_share_path,
get_package_share_directory,
)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    PythonExpression,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import xacro
from launch.actions import OpaqueFunction


# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def evaluate_xacro(context, *args, **kwargs):

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('odometry_calibration'), 'urdf', 'calibration.urdf.xacro')

    #robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = xacro.process_file(xacro_file, 
            mappings={  
                }).toxml()

    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace='tripod', 
      name='robot_state_publisher_tripod',
        output='both',
      parameters=[{
        'robot_description': robot_description_config
      }])

    return [robot_state_publisher_node]


def generate_launch_description():

    description_path = get_package_share_path('odometry_calibration')
    default_rviz_config_path = description_path / 'config/calibration.rviz'


    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')


    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_tripod',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_tripod_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_tripod',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    polygon_publisher_node = Node(
        package='odometry_calibration',
        executable='polygon_publisher',
        name='polygon_publisher',
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        #robot_state_publisher_node,
        OpaqueFunction(function=evaluate_xacro),
        rviz_node,
        polygon_publisher_node
    ])
