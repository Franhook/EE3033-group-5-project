import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'balance_object_detect'
setup(
    name=package_name, version='0.0.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='wheeltec', maintainer_email='wheeltec@todo.todo',
    description='Combined YOLO detection, confirmation and target localization',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'combined_target = balance_object_detect.combined_target_node:main',
    ]},
)
