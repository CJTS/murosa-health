import os
from glob import glob
from setuptools import setup

package_name = 'murosa_plan_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos',
    maintainer_email='carlosjoel.tavares@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = murosa_plan_patrol.planner_node:main',
            'coordinator = murosa_plan_patrol.coordinator_node:main',
            'robot = murosa_plan_patrol.robot_node:main',
            'environment = murosa_plan_patrol.environment_node:main',
        ],
    },
)
