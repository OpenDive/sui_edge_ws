from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sui_indexer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include Prisma schema
        (os.path.join('share', package_name, 'prisma'), glob('prisma/*.prisma')),
    ],
    install_requires=[
        'setuptools',
        'prisma',  # Prisma Client Python
        'pysui',   # Sui Python SDK
        'python-dotenv',  # For environment configuration
        'asyncio',  # For async support
    ],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='A ROS2 package for indexing Sui blockchain events',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'indexer_node = sui_indexer.indexer_node:main',
            'prisma_setup = sui_indexer.scripts.setup_prisma:main',
        ],
    },
)
