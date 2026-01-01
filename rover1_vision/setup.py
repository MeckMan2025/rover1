from setuptools import setup
import os
from glob import glob

package_name = 'rover1_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover1 Project',
    maintainer_email='rover1@example.com',
    description='Vision-based behaviors for rover1 using Hailo-8L AI accelerator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dog_follower = rover1_vision.dog_follower:main',
        ],
    },
)
