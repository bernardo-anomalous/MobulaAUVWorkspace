from setuptools import setup, find_packages

package_name = 'auv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mobula AUV Team',
    maintainer_email='dev@mobula.local',
    description='Portable simulator nodes for GUI development without hardware.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_sim = auv_sim.motion_sim_node:main',
            'depth_sim = auv_sim.depth_sim_node:main',
            'battery_system_sim = auv_sim.battery_system_sim_node:main',
            'servo_sim = auv_sim.servo_sim_node:main',
        ],
    },
)
