import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'balance_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EE3033 group',
    maintainer_email='todo@todo.todo',
    description='Autonomous search, target localization and return mission',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_confirmation_node = '
            'balance_mission.target_confirmation_node:main',
            'target_localization_node = '
            'balance_mission.target_localization_node:main',
            'target_map_node = balance_mission.target_map_node:main',
            'mission_manager_node = '
            'balance_mission.mission_manager_node:main',
            'mapping_supervisor_node = '
            'balance_mission.mapping_supervisor_node:main',
            'integrated_frontier_explorer = balance_mission.integrated_frontier_explorer:main',
        ],
    },
)
