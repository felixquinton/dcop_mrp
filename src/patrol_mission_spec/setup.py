import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

packages = find_packages(exclude=['test'])

package_name = 'patrol_mission_spec'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Data team spec files
        (os.path.join('share', package_name, 'data', 'current_spec'),
         glob('data/current_spec/*')),
        (os.path.join('share', package_name, 'data', 'mission_spec'),
         glob('data/mission_spec/*')),
        (os.path.join('share', package_name, 'data', 'robots_spec'),
         glob('data/robots_spec/*'))
         ]


for sc_id in range(75):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'grid_graphs_dim=(5, 5)_obsprop=0.25', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/grid_graphs_dim=(5, 5)_obsprop=0.25/scenario_{sc_id}/*.json'))]

for sc_id in range(75):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'grid_graphs_dim=(10, 10)_obsprop=0.25', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/grid_graphs_dim=(10, 10)_obsprop=0.25/scenario_{sc_id}/*.json'))]

for sc_id in range(75):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'star_graphs_nb_branch=10_branch_length=5_obsprop=0.25', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/star_graphs_nb_branch=10_branch_length=5_obsprop=0.25/scenario_{sc_id}/*.json'))]

for sc_id in range(75):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'star_graphs_nb_branch=5_branch_length=5_obsprop=0.25', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/star_graphs_nb_branch=5_branch_length=5_obsprop=0.25/scenario_{sc_id}/*.json'))]

for sc_id in range(75):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'star_graphs_nb_branch=10_branch_length=10_obsprop=0.25', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/star_graphs_nb_branch=10_branch_length=10_obsprop=0.25/scenario_{sc_id}/*.json'))]

for sc_id in range(50):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'caylus_graphs_large', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/caylus_graphs_large/scenario_{sc_id}/*.json'))]

for sc_id in range(65):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'caylus_graphs_small', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/caylus_graphs_small/scenario_{sc_id}/*.json'))]

for sc_id in range(10):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'caylus_graphs_large_events_from_600_to_900', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/caylus_graphs_large_events_from_600_to_900/scenario_{sc_id}/*.json'))]

for sc_id in range(10):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'caylus_graphs_large_events_from_900_to_1200', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/caylus_graphs_large_events_from_900_to_1200/scenario_{sc_id}/*.json'))]

for sc_id in range(10):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'caylus_graphs_large_events_from_1200_to_1800', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/caylus_graphs_large_events_from_1200_to_1800/scenario_{sc_id}/*.json'))]

for sc_id in range(50):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'caylus_graphs_large_events_from_1200_to_1500', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/caylus_graphs_large_events_from_1200_to_1500/scenario_{sc_id}/*.json'))]

for sc_id in range(50):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'grid_graphs_dim=(7, 7)_obsprop=0.15', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_{sc_id}/*.json'))]

for sc_id in range(50):
        data_files += [(os.path.join('share', package_name, 'data', 'generated_scenarios', 'star_graphs_nb_branch=10_branch_length=5_obsprop=0.15', f'scenario_{sc_id}'), glob(f'data/generated_scenarios/star_graphs_nb_branch=10_branch_length=5_obsprop=0.15/scenario_{sc_id}/*.json'))]
        
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fquinton',
    maintainer_email='fe.quinton@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
