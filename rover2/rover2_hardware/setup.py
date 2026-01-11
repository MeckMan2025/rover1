from setuptools import setup

package_name = 'rover2_hardware'

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
    maintainer='Andrew Meckley',
    maintainer_email='andrew.meckley@example.com',
    description='Pack Robot hardware drivers - motors, battery, kinematics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hiwonder_driver = rover2_hardware.hiwonder_driver:main',
            'mecanum_kinematics = rover2_hardware.mecanum_kinematics:main',
            'battery_monitor = rover2_hardware.battery_monitor:main',
        ],
    },
)