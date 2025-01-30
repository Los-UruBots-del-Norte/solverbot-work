from setuptools import setup
import os
from glob import glob

package_name = 'motor_board_md49_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Urubots',
    maintainer_email='urubots@utec.ed.uy',
    description='Controlador para MotorBoard MD49 en ROS 2',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_board_node = motor_board_md49_ros2.ros_interface:main',
        ],
    },
)
