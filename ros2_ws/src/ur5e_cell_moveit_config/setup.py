from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_cell_moveit_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='withanage',
    maintainer_email='wcjayashan@gmail.com',
    description='MoveIt config for UR5e cell using moveit_configs_utils',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
