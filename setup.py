import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rov_thruster_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Hadhin T A',
    maintainer_email='user@example.com',
    description='Underwater ROV motor control firmware simulator and ROS 2 bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_bridge_node = rov_thruster_sim.thruster_bridge_node:main',
        ],
    },
)