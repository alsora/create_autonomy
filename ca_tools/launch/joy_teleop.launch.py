import os
from launch import LaunchDescription
import launch_ros.actions
import launch.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  joy_dev = launch.substitutions.LaunchConfiguration('joy_dev', default="/dev/input/js0")
  joy_config = launch.substitutions.LaunchConfiguration('joy_config', default="xbox360")

  ca_tools_share_path = get_package_share_directory('ca_tools')
  default_config_file = os.path.join(ca_tools_share_path,
                                            'config',
                                            joy_config + '.yaml')

  teleop_config = launch.substitutions.LaunchConfiguration('teleop_config', default=default_config_file)


  return LaunchDescription([

    launch_ros.actions.Node(
      package='joy',
      node_executable='joy_node',
      node_name='joy_node',
      output='screen',
      parameters=[teleop_config, {'dev': joy_dev}, {'deadzone': 0.2}, {'autorepeat_rate': 20}]),

])
