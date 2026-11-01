from glob import glob
from setuptools import find_packages, setup

package_name = 'lab1_turtle'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lab1 Team',
    maintainer_email='team@example.com',
    description='Lab 1: ROS 2 turtlesim actions and services',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_pose_server = lab1_turtle.go_to_pose_server:main',
            'reset_service = lab1_turtle.reset_service:main',
            'square_action_server = lab1_turtle.square_action_server:main',
        ],
    },
)
