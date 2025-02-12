import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mail-delivery-robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['src.sensors.camera_sensor',
                'src.sensors.beacon_sensor',
                'src.sensors.bumper_sensor',
                'src.sensors.lidar_sensor',
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
                'src.tools.nav_parser'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'config'), glob(os.path.join('src/config', '*.csv'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='create3',
    maintainer_email='deniscengu@cmail.carleton.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
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
            'captain = src.control.captain:main'
        ],
    },
)
