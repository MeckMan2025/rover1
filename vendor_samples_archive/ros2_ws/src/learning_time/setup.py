from setuptools import find_packages, setup

package_name = 'learning_time'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='1111@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'rate_demo=learning_time.rate_demo:main',
        'Timer_demo=learning_time.Timer_demo:main',        
        'get_clock_demo=learning_time.get_clock_demo:main',  
        'TimeDuration_demo=learning_time.TimeDuration_demo:main', 
        ],
    },
)
