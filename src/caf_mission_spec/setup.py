import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

packages = find_packages(exclude=['test'])

package_name = 'caf_mission_spec'

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Data team spec files
        (os.path.join('share', package_name, 'data', 'current_spec'),
         glob('data/current_spec/*')),
        (os.path.join('share', package_name, 'data', 'current_spec'),
         glob('data/mission_spec/*')),
        (os.path.join('share', package_name, 'data', 'current_spec'),
         glob('data/robots_spec/default_robots_spec.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='amilot@laas.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
