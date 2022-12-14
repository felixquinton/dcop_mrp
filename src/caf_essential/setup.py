from setuptools import setup
import os
from glob import glob

package_name = 'caf_essential'

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
    maintainer='antoine',
    maintainer_email='amilot@laas.fr',
    description='CAF package for essential scripts and nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'caf_base_node = caf_essential.base_node:main',
        ],
    },
)
