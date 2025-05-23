from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sui_edge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config',
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'pysui>=0.84.0'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='ROS2 bridge for Sui blockchain integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sui_edge_bridge_node = sui_edge.bridge_node:main',
            'sui_edge_sui_service_node = sui_edge.sui_service_node:main',
        ],
    },
    scripts=[],
    package_dir={'': '.'},
    package_data={},
) 