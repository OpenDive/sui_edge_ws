from setuptools import find_packages, setup

package_name = 'sui_edge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/bridge_launch.py']),
        ('share/' + package_name + '/config',
            ['config/bridge_config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'sui-sdk',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='Marcus.Arnett10@gmail.com',
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
