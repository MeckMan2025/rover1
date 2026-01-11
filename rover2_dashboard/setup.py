from setuptools import setup
import os
from glob import glob

package_name = 'rover2_dashboard'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('static/*.html')),
        ('share/' + package_name + '/static/css', glob('static/css/*.css')),
        ('share/' + package_name + '/static/js', glob('static/js/*.js')),
        ('lib/' + package_name, ['scripts/rover2_web_dashboard']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Rover2 Project',
    maintainer_email='rover2@example.com',
    description='Web dashboard for Rover2 pack robot monitoring and person-following control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover2_web_dashboard = rover2_dashboard.web_dashboard:main',
        ],
    },
)