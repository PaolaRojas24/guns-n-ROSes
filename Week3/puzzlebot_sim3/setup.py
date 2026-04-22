from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim3'

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
            'trayectory_node = puzzlebot_sim3.trayectory_node:main',
            'PointStabilisation_node = puzzlebot_sim3.point_stabilisation_control:main',
            'localisation = puzzlebot_sim3.localisation:main',
            'joint_state_pub = puzzlebot_sim3.joint_state_pub:main',
            'puzzlebot_sim = puzzlebot_sim3.puzzlebot_sim:main',
            'coords_transform = puzzlebot_sim3.coords_transform:main',
        ],
    },
)
