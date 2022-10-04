import os
from glob import glob
from setuptools import setup

package_name = 'caf_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fquinton',
    maintainer_email='fe.quinton@gmail.com',
    description='CAF robots package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'comsim_node = caf_robot.comsim_member_function:main',
                'robot_node = caf_robot.robot_member_function:main',
        ],
    },
)
