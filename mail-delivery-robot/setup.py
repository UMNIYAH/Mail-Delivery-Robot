import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mail-delivery-robot'

setup(
    name=package_name,
    version='0.0.0',
<<<<<<< HEAD
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'config'),
            glob(os.path.join('src/config', '*.csv'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
=======
    packages=find_packages(exclude=['test']),
    py_modules=['src.sensors.camera_sensor',
                'src.sensors.beacon_sensor',
                'src.sensors.bumper_sensor',
                'src.sensors.lidar_sensor',
                'src.sensors.navigation_unit',
                'src.sensors.intersection_detection_unit',
                'src.sensors.battery_monitor',
                'src.layers.avoidance_layer',
                'src.layers.docking_layer',
                'src.layers.travel_layer',
                'src.layers.turning_layer',
                'src.communication.client',
                'src.communication.music_player',
                'src.control.captain',
                'src.control.action_translator',
                'src.tools.csv_parser',
                'src.tools.map',
                'src.tools.nav_parser',
                'src.tests.travel_analyzer'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'config'), glob(os.path.join('src/config', '*.csv'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
>>>>>>> 21be50cdcbfd885d0b3dae9febdc13d01280c0e7
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create3',
    maintainer_email='deniscengu@cmail.carleton.ca',
<<<<<<< HEAD
    description='Carleton mail delivery robot',
=======
    description='TODO: Package description',
>>>>>>> 21be50cdcbfd885d0b3dae9febdc13d01280c0e7
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'camera_sensor = sensors.camera_sensor:main',
            'beacon_sensor = sensors.beacon_sensor:main',
            'bumper_sensor = sensors.bumper_sensor:main',
            'lidar_sensor = sensors.lidar_sensor:main',
            'avoidance_layer = layers.avoidance_layer:main',
            'docking_layer = layers.docking_layer:main',
            'travel_layer = layers.travel_layer:main',
            'turning_layer = layers.turning_layer:main',
            'client = communication.client:main',
            'music_player = communication.music_player:main',
            'captain = control.captain:main',
            'travel_analyzer = tests.travel_analyzer:main',
            'navigation_unit = sensors.navigation_unit:main',
            'intersection_detection_unit = sensors.intersection_detection_unit:main',
            'battery_monitor = sensors.battery_monitor:main',
            'csv_parser = tools.csv_parser:main',
            'map = tools.map:main',
            'nav_parser = tools.nav_parser:main',
        ],
    },
)

=======
            'camera_sensor = src.sensors.camera_sensor:main',
            'beacon_sensor = src.sensors.beacon_sensor:main',
            'bumper_sensor = src.sensors.bumper_sensor:main',
            'lidar_sensor = src.sensors.lidar_sensor:main',
            'avoidance_layer = src.layers.avoidance_layer:main',
            'docking_layer = src.layers.docking_layer:main',
            'travel_layer = src.layers.travel_layer:main',
            'turning_layer = src.layers.turning_layer:main',
            'client = src.communication.client:main',
            'music_player = src.communication.music_player:main',
            'captain = src.control.captain:main',
            'travel_analyzer = src.tests.travel_analyzer:main',
            'navigation_unit = src.sensors.navigation_unit:main',
            'intersection_detection_unit = src.sensors.intersection_detection_unit:main',
            'battery_monitor = src.sensors.battery_monitor:main'
        ],
    },
)
>>>>>>> 21be50cdcbfd885d0b3dae9febdc13d01280c0e7
