from setuptools import setup
import os
from glob import glob

package_name = 'gnss_health_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover1 Project',
    maintainer_email='rover1@example.com',
    description='GNSS Health Monitor - Aggregates GPS/RTK/NTRIP status for Foxglove dashboards',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_health_monitor_node = gnss_health_monitor.gnss_health_monitor_node:main',
        ],
    },
)