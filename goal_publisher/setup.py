import os
from glob import glob
from setuptools import setup
package_name = 'goal_publisher'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A package to detect goal markers and publish their position.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher_node = goal_publisher.goal_publisher_node:main',
            'explorer_node = goal_publisher.explorer_node:main',
        ],
    },
)
