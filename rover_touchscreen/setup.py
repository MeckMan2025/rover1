from setuptools import setup

package_name = 'rover_touchscreen'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/touchscreen.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='rover@example.com',
    description='Simple PyQt touchscreen dashboard for 7" rover display',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'touchscreen_dashboard = rover_touchscreen.touchscreen_dashboard:main',
        ],
    },
)