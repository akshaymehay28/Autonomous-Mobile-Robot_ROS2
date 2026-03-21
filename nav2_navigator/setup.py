from setuptools import setup
import os
from glob import glob

package_name = 'nav2_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'behavior_trees'), glob('config/behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Nav2 navigator for maze traversal',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_node = nav2_navigator.navigator_node:main',
            'map_relay_node = nav2_navigator.map_relay_node:main',
        ],
    },
)
