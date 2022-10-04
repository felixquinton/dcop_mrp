import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

packages = find_packages(exclude=['test'])

package_name = 'patrol_sim_robots'
setup(
    name=package_name,
    version='0.0.0',
    #packages=[package_name],
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
    maintainer='fquinton',
    maintainer_email='fe.quinton@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_sim_robots_node = patrol_sim_robots.sim_robots_function:main',
        ],
    },
)
