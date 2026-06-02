from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paola Rojas',
    maintainer_email='a01737136@tec.mx',
    description='Puzzlebot Kinematic Sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation = final_challenge.localisation:main',
            'puzzlebot_sim = final_challenge.puzzlebot_sim:main',
            'mission_node = final_challenge.mission_node:main',
            'planner_node = final_challenge.planner_node:main',
            'controller_node = final_challenge.controller_node:main',
            'slam_resetter = final_challenge.slam_resetter_node:main',
            'camera_node = final_challenge.camera_node:main',
            'bug1_node = final_challenge.bug1_node:main',
            'control_node = final_challenge.control_node:main',
        ],
    },
)
