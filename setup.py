from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frontier'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.py'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Documentation
        (os.path.join('share', package_name, 'docs'),
            glob(os.path.join('docs', '*.md'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user1',
    maintainer_email='kikiws70@gmail.com',
    description='FRONTIER: Frustum-based Refined Object Networking & Tracking via Intersection Engine for Ranging-data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_node = frontier.frontier_node:main',
            'frustum_visualizer_node = frontier.frustum_visualizer_node:main',
            'test_frustum_generator = frontier.test_frustum_generator:main',
            'test_intersection_engine = frontier.test_intersection_engine:main',
            'test_visualization = frontier.test_visualization:main',
        ],
    },
)
