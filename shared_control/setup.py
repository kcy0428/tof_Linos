from setuptools import find_packages, setup
from glob import glob

package_name = 'shared_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chan',
    maintainer_email='kochanyeong123@gmail.com',
    description='Shared-control obstacle avoidance for TurtleBot3 Waffle.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_bridge = shared_control.tof_bridge_node:main',
            'obstacle_detector = shared_control.obstacle_detector_node:main',
            'gap_analyzer = shared_control.gap_analyzer_node:main',
            'avoidance = shared_control.avoidance_node:main',
            'arbitrator = shared_control.arbitrator_node:main',
        ],
    },
)
