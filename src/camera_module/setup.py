from setuptools import find_packages, setup

package_name = 'camera_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'camera_module',
        'camera_module.dynamixels',
    ],
    # packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "servo_node = camera_module.servo_ros:main",
            "dynamixel_node = camera_module.dynamixels.ros_dynamixel:main",
            "camera_publisher = camera_module.camera_publisher:main",
            "camera_viewer = camera_module.camera_viewer:main",
        ],
    },
)
