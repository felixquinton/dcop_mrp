from setuptools import setup
import os
from glob import glob

package_name = 'caf_interface'

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
    description='CAF package for the GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'interface_node = caf_interface.interface_function:main',
        ],
    },
)
