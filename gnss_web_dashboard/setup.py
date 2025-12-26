from setuptools import setup
import os
from glob import glob

package_name = 'gnss_web_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('static/*')),
        ('lib/' + package_name, ['scripts/gnss_web_dashboard']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Rover1 Project',
    maintainer_email='rover1@example.com',
    description='Simple web dashboard for GNSS health monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_web_dashboard = gnss_web_dashboard.web_dashboard:main',
        ],
    },
)