import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  ca_driver_share_path = get_package_share_directory('ca_driver')
  default_config_file = os.path.join(ca_driver_share_path,
                                            'config',
                                            'default.yaml')

  #ca_description_share_path = get_package_share_directory('ca_description')
  #create1_description_launch = os.path.join(ca_description_share_path,
  #                                          'launch',
  #                                          'roomba_400.launch.py')

  config_file = launch.substitutions.LaunchConfiguration('config', default=default_config_file)
  #load_description = launch.substitutions.LaunchConfiguration('desc', default='true')

  return LaunchDescription([

    launch_ros.actions.Node(
      package='ca_driver',
      node_executable='create_driver_main',
      node_name='create_driver_main',
      output='screen',
      parameters=[config_file, { 'robot_model': "ROOMBA_400"}]),

    #IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([create1_description_launch])
    #)

  ])