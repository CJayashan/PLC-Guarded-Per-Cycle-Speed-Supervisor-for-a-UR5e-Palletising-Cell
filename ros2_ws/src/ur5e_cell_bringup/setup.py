from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur5e_cell_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur5e_cell_bringup']),
        ('share/ur5e_cell_bringup', ['package.xml']),
        ('share/ur5e_cell_bringup/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/ur5e_cell_bringup/config', glob(os.path.join('config', '*'))),
        ('share/ur5e_cell_bringup/worlds', glob(os.path.join('worlds', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chathu',
    maintainer_email='you@example.com',
    description='Bringup for the UR5e cell (fake & gazebo)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cycle_signals_node = ur5e_cell_bringup.cycle_signals_node:main',
            'cycle_signals_node_rewrite = ur5e_cell_bringup.cycle_signals_node_rewrite:main',
            'speed_supervisor_node = ur5e_cell_bringup.speed_supervisor_node:main',
            'moveit_speed_executor_node = ur5e_cell_bringup.moveit_speed_executor_node:main',
            'cycle_logger_node = ur5e_cell_bringup.cycle_logger_node:main',
            'rule_based_supervisor_node = ur5e_cell_bringup.rule_based_supervisor_node:main',
            'bayesian_speed_supervisor_node = ur5e_cell_bringup.bayes_supervisor_node:main',
            'ppo_speed_supervisor_node = ur5e_cell_bringup.ppo_speed_supervisor_node:main',
            'torque_estimator_node = ur5e_cell_bringup.sim_torque_estimator_node:main',

            # (any other nodes here)
        ],
    },
)
