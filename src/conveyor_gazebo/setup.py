import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'conveyor_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vanhdeptraibodoiqua',
    maintainer_email='mgmgd2004@gmail.com',
    description='Gazebo worlds and launch for the conveyor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

        ],
    },
)
