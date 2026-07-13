import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'conveyor_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vanhdeptraibodoiqua',
    maintainer_email='mgmgd2004@gmail.com',
    description='Conveyor belt URDF/xacro description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

        ],
    },
)
