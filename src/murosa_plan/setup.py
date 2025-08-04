import os
from glob import glob
from setuptools import setup

package_name = 'murosa_plan'

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
    maintainer_email='user@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment = murosa_plan.environment_node:main',
            'patrol_environment = murosa_plan.patrol_environment_node:main',
            'disinfect_environment = murosa_plan.disinfect_environment_node:main',
            'planner = murosa_plan.planner_node:main',
            'patrol_planner = murosa_plan.patrol_planner_node:main',
            'disinfect_planner = murosa_plan.disinfect_planner_node:main',
            'arm = murosa_plan.agents.arm_node:main',
            'nurse = murosa_plan.agents.nurse_node:main',
            'robot = murosa_plan.agents.robot_node:main',
            'patrol = murosa_plan.agents.patrol_node:main',
            'nurse_disinfect = murosa_plan.agents.nurse_disinfect_node:main',
            'spotrobot = murosa_plan.agents.spotrobot_node:main',
            'uvdrobot = murosa_plan.agents.uvdrobot_node:main',

        ],
    },
)
