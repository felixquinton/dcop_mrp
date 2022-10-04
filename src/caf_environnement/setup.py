from setuptools import setup

package_name = 'caf_environment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fquinton',
    maintainer_email='fe.quinton@gmail.com',
    description='CAF package for obstacles and other environment objects and properties',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'obstacle = caf_environment.obstacle_member_function:main',
        ],
    },
)
