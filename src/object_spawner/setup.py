import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'object_spawner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vanhdeptraibodoiqua',
    maintainer_email='mgmgd2004@gmail.com',
    description='Randomly spawn colored cubes onto the conveyor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawner_node = object_spawner.spawner_node:main'
        ],
    },
)
