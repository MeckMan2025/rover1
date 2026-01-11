from setuptools import setup
import os
from glob import glob

package_name = 'rover2_vision'

setup(
    name=package_name,
    version='2.0.0',
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
    maintainer='Rover2 Project',
    maintainer_email='rover2@example.com',
    description='Person detection and following for rover2 pack robot using Hailo-8L AI accelerator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_follower = rover2_vision.person_follower:main',
        ],
    },
)
