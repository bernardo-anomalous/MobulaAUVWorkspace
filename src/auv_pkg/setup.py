from setuptools import find_packages, setup

package_name = 'auv_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='b',
    maintainer_email='b@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "keyboard_control = auv_pkg.keyboard_control:main",
            "servo_interpolation = auv_pkg.servo_interpolation:main",
            "servo_driver = auv_pkg.servo_driver:main",
            "imu_node = auv_pkg.imu_node:main",
            "pitch_pid = auv_pkg.pitch_pid:main",
            "roll_pid = auv_pkg.roll_pid:main",
            "keyboard_control_swim = auv_pkg.keyboard_control_swim:main",
            "camera_control = auv_pkg.camera_control:main",
            "depth_sensor = auv_pkg.depth_node:main",
            "acceleration_node = auv_pkg.acceleration_node:main",
            "battery_node = auv_pkg.battery_node:main",
            "system_monitor_node = auv_pkg.system_monitor_node:main",
            "console_bridge_node = auv_pkg.console_bridge_node:main",
        ],
    },
)
