import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

packages = find_packages(exclude=['test'])

package_name = 'caf_sim_robots'
setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=packages,
    # Utils files
    # package_dir={'': 'utils'},
    # Files to install
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Data files
        # (os.path.join('share', package_name, 'data'), glob('data/**/*')),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='amilot@laas.fr',
    description='Simulation of robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['sim_robots_node = caf_sim_robots.sim_robots_function:main',
    ],
    },
)
