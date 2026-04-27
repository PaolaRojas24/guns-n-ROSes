from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multiple_sim'

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
            'trayectory_node = multiple_sim.trayectory_node:main',
            'PointStabilisation_node = multiple_sim.point_stabilisation_control:main',
            'localisation = multiple_sim.localisation:main',
            'joint_state_pub = multiple_sim.joint_state_pub:main',
            'puzzlebot_sim = multiple_sim.puzzlebot_sim:main',
            'pursuit_node = multiple_sim.pursuit_node:main',
            'coords_transform = multiple_sim.coords_transform:main',
        ],
    },
)
