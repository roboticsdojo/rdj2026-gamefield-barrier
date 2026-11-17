import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gamefield_barrier'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='screen',
    maintainer_email='screen@example.com',
    description='ROS 2 node for the automated gamefield barrier',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barrier_node = gamefield_barrier.barrier_node:main',
        ],
    },
)