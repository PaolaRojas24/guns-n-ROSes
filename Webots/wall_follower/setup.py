from setuptools import find_packages, setup

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_follower.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/E-puck.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/world_maze.wbt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Itzel Hernández',
    maintainer_email='a01737526@tec.mx',
    description='TF2_Examples',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = wall_follower.wall_follower:main',
        ],
    },
)
