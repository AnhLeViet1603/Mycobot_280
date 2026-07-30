from setuptools import find_packages, setup

package_name = 'mycobot_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vanhdeptraibodoiqua',
    maintainer_email='mgmgd2004@gmail.com',
    description='Demo nodes cho myCobot 280 (test trajectory, grasp, pick-and-place)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_arm = mycobot_demo.test_arm_trajectory:main',
            'test_gripper = mycobot_demo.test_gripper:main',
        ],
    },
)
