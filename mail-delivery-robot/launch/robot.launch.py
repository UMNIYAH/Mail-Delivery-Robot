from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'
        ),
        Node(
            package='mail-delivery-robot',
            namespace='sensors',
            executable='camera_sensor',
            name='camera_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            namespace='sensors',
            executable='lidar_sensor',
            name='lidar_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            namespace='sensors',
            executable='bumper_sensor',
            name='bumper_sensor'
        ),
        Node(
            package='mail-delivery-robot',
            namespace='sensors',
            executable='beacon_sensor',
            name='beacon_sensor'
        )
   ])