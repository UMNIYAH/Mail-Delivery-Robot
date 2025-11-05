import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mail_delivery_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'config'), glob(os.path.join('src', package_name, 'config', '*.csv'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create3',
    maintainer_email='deniscengu@cmail.carleton.ca',
    description='Carleton mail delivery robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_sensor = mail_delivery_robot.sensors.camera_sensor:main',
            'beacon_sensor = mail_delivery_robot.sensors.beacon_sensor:main',
            'bumper_sensor = mail_delivery_robot.sensors.bumper_sensor:main',
            'lidar_sensor = mail_delivery_robot.sensors.lidar_sensor:main',
            'avoidance_layer = mail_delivery_robot.layers.avoidance_layer:main',
            'docking_layer = mail_delivery_robot.layers.docking_layer:main',
            'travel_layer = mail_delivery_robot.layers.travel_layer:main',
            'turning_layer = mail_delivery_robot.layers.turning_layer:main',
            'client = mail_delivery_robot.communication.client:main',
            'music_player = mail_delivery_robot.communication.music_player:main',
            'captain = mail_delivery_robot.control.captain:main',
            'travel_analyzer = mail_delivery_robot.tests.travel_analyzer:main',
            'navigation_unit = mail_delivery_robot.sensors.navigation_unit:main',
            'intersection_detection_unit = mail_delivery_robot.sensors.intersection_detection_unit:main',
            'battery_monitor = mail_delivery_robot.sensors.battery_monitor:main',
            'csv_parser = mail_delivery_robot.tools.csv_parser:main',
            'map = mail_delivery_robot.tools.map:main',
            'ai_command = mail_delivery_robot.control.ai_command:main',
            'nav_parser = mail_delivery_robot.tools.nav_parser:main',
            'ai_processor_node = mail_delivery_robot.control.ai_processor_node:main'
        ],
    },
)
