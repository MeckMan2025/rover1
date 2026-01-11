from setuptools import setup
import os
from glob import glob

package_name = 'rover2_hardware'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='Andrew Meckley',
    maintainer_email='andrew@example.com',
    description='Hardware abstraction layer for Rover2 pack robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'berry_imu_driver = rover2_hardware.berry_imu_driver:main',
            'hiwonder_driver = rover2_hardware.hiwonder_driver:main',
            'mecanum_kinematics = rover2_hardware.mecanum_kinematics:main',
            'battery_monitor = rover2_hardware.battery_monitor:main',
            'stadia_teleop = rover2_hardware.stadia_teleop:main',
            'fix_to_nmea = rover2_hardware.fix_to_nmea:main',
        ],
    },
)
