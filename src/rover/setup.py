import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='ryan@rkuederle.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_node = rover.motor_node:main",
            "motor_gui = rover.drive.motor_gui:main",
            "twist_control = rover.drive.twist_control_node:main",
            "joy_twist = rover.drive.joy_twist_node:main",
            "esc_status = rover.drive.esc_status_node:main",
            "joint_state_pub = rover.drive.joint_state_pub:main",
            "nav_vis_pub = rover.vision.nav_vis_pub:main",
            "flow = rover.vision.flow:main",
        ],
    },
)
