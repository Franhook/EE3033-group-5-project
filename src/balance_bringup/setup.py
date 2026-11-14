import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'balance_bringup'


setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        # Register the package in the ament resource index.
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # Install package.xml.
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Install all Python launch files.
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='balance',
    maintainer_email='balance@example.com',

    description=(
        'Combined bringup launch files for the '
        'self-balancing robot.'
    ),

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            (
                'continuous_waypoint_loop = balance_bringup.continuous_waypoint_loop:main'
            ),
        ],
    },
)
