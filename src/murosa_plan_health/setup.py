import os
from glob import glob
from setuptools import setup

package_name = 'murosa_plan_health'

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
            'environment = murosa_plan_health.environment_node:main',
            'planner = murosa_plan_health.planner_node:main',
            'coordinator = murosa_plan_health.agents.coordinator_node:main',
            'arm = murosa_plan_health.agents.arm_node:main',
            'nurse = murosa_plan_health.agents.nurse_node:main',
            'robot = murosa_plan_health.agents.robot_node:main',
        ],
    },
)
