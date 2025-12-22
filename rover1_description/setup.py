import os
from glob import glob
from setuptools import setup

package_name = 'rover1_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Meckley',
    maintainer_email='andrew@example.com',
    description='URDF Description for Rover1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
