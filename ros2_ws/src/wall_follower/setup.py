from setuptools import setup
import os
from glob import glob

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joe Lin',
    maintainer_email='weichih.lin@protonmail.com',
    description='ROS2 basic concept with ROSKY2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'wall_following = {package_name}.wall_following:main',
            f'find_wall_service = {package_name}.find_wall_service:main',
            f'test_server = {package_name}.test_server:main',
            f'test_client = {package_name}.test_client:main',
            f'test_action = p{package_name}.test_action:main',
            f'record_odometry_action_server = {package_name}.record_odometry_action_server:main',
        ],
    },
)
