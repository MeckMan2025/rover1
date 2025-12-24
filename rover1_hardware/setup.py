from setuptools import setup
import os
from glob import glob

package_name = 'rover1_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Meckley',
    maintainer_email='andrew@example.com',
    description='Hardware abstraction layer for Rover1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'berry_imu_driver = rover1_hardware.berry_imu_driver:main',
            'hiwonder_driver = rover1_hardware.hiwonder_driver:main',
            'mecanum_kinematics = rover1_hardware.mecanum_kinematics:main',
            'battery_monitor = rover1_hardware.battery_monitor:main',
            'stadia_teleop = rover1_hardware.stadia_teleop:main',
            'fix_to_nmea = rover1_hardware.fix_to_nmea:main',
        ],
    },
)
