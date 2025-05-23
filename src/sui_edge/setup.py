from setuptools import setup
import os
from glob import glob

package_name = 'sui_edge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('lib', package_name), [])  # This ensures the lib/sui_edge directory is created
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
            'bridge_node = sui_edge.bridge_node:main',
            'sui_service_node = sui_edge.sui_service_node:main',
        ],
    },
) 