from setuptools import setup
import os
from glob import glob
package_name = 'pkg_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.xml'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_demo = pkg_topic.publisher_demo:main',
            'subscriber_demo = pkg_topic.subscriber_demo:main'
        ],
    },
)
