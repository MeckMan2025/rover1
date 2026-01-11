from setuptools import setup
import os
from glob import glob

package_name = 'rover2_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'), glob('static/**/*', recursive=True)),
        (os.path.join('share', package_name, 'static/css'), glob('static/css/*')),
        (os.path.join('share', package_name, 'static/js'), glob('static/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Meckley',
    maintainer_email='andrew.meckley@example.com',
    description='Pack Robot web dashboard - minimal interface with mandatory teleop',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pack_robot_dashboard = rover2_dashboard.pack_robot_dashboard:main',
        ],
    },
)