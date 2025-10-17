import os
from glob import glob
from setuptools import setup

package_name = 'agents'

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
    maintainer='I',
    maintainer_email='user@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm = agents.nodes.arm_node:main',
            'collector = agents.nodes.collector_node:main',
            'environment = agents.nodes.environment_node:main',
            'navigator = agents.nodes.navigator_node:main',
            'nurse = agents.nodes.nurse_node:main',
            'spot = agents.nodes.spot_node:main',
            'uvd = agents.nodes.uvd_node:main',
        ],
    },
)
